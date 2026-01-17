#include "FBXLoader.h"

#include <cstdio>
#include <memory>
#include <vector>
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <cmath>

#ifdef RT_ENABLE_FBX
#include <fbxsdk.h>
#include "objects.h"
#include "materials.h"
#include "texture.h"

namespace
{
struct FBXScopedScene
{
    FbxManager* manager = nullptr;
    FbxScene* scene = nullptr;

    ~FBXScopedScene() { Reset(); }

    void Reset()
    {
        if (scene) {
            scene->Destroy();
            scene = nullptr;
        }
        if (manager) {
            manager->Destroy();
            manager = nullptr;
        }
    }
};

static bool LoadSceneInternal(const char* filename, FBXScopedScene& out)
{
    out.Reset();
    out.manager = FbxManager::Create();
    if (!out.manager) return false;

    FbxIOSettings* ios = FbxIOSettings::Create(out.manager, IOSROOT);
    out.manager->SetIOSettings(ios);

    FbxImporter* importer = FbxImporter::Create(out.manager, "");
    if (!importer->Initialize(filename, -1, out.manager->GetIOSettings())) {
        importer->Destroy();
        out.Reset();
        return false;
    }

    out.scene = FbxScene::Create(out.manager, "scene");
    if (!out.scene) {
        importer->Destroy();
        out.Reset();
        return false;
    }

    const bool ok = importer->Import(out.scene);
    importer->Destroy();
    if (!ok) {
        out.Reset();
        return false;
    }

    FbxGeometryConverter converter(out.manager);
    converter.Triangulate(out.scene, true);
    return true;
}

static Matrix34f ToMatrix34(const FbxAMatrix& m)
{
    Matrix34f out;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 3; ++r) {
            out.cell[c * 3 + r] = static_cast<float>(m.Get(r, c));
        }
    }
    return out;
}

static Vec3f TransformDir(const FbxAMatrix& m, const Vec3f& dir)
{
    return Vec3f(
        float(m.Get(0, 0) * dir.x + m.Get(0, 1) * dir.y + m.Get(0, 2) * dir.z),
        float(m.Get(1, 0) * dir.x + m.Get(1, 1) * dir.y + m.Get(1, 2) * dir.z),
        float(m.Get(2, 0) * dir.x + m.Get(2, 1) * dir.y + m.Get(2, 2) * dir.z)
    );
}

static Color ReadColor(FbxProperty prop, const Color& fallback)
{
    if (prop.IsValid()) {
        FbxDouble3 c = prop.Get<FbxDouble3>();
        return Color(float(c[0]), float(c[1]), float(c[2]));
    }
    return fallback;
}

static TextureFile* LoadTextureFile(TextureFileList& texFiles, const std::string& filename, const std::string& baseDir)
{
    if (filename.empty()) return nullptr;
    std::filesystem::path p(filename);
    if (!p.is_absolute() && !baseDir.empty()) {
        p = std::filesystem::path(baseDir) / p;
    }
    const std::string full = p.lexically_normal().string();

    if (TextureFile* existing = dynamic_cast<TextureFile*>(texFiles.Find(full.c_str()))) return existing;

    auto* tex = new TextureFile;
    tex->SetName(full.c_str());
    if (!tex->LoadFile()) {
        std::printf("ERROR: cannot load texture file %s\n", full.c_str());
        delete tex;
        return nullptr;
    }
    texFiles.push_back(tex);
    return tex;
}

static TextureMap* LoadTextureMap(FbxProperty prop, TextureFileList& texFiles, const std::string& baseDir)
{
    if (!prop.IsValid()) return nullptr;
    const int texCount = prop.GetSrcObjectCount<FbxFileTexture>();
    if (texCount <= 0) return nullptr;
    FbxFileTexture* tex = prop.GetSrcObject<FbxFileTexture>(0);
    if (!tex) return nullptr;
    const char* fn = tex->GetFileName();
    if (!fn || fn[0] == '\0') fn = tex->GetRelativeFileName();
    if (!fn || fn[0] == '\0') return nullptr;
    if (TextureFile* tf = LoadTextureFile(texFiles, fn, baseDir)) {
        return new TextureMap(tf);
    }
    return nullptr;
}

static float RoughnessFromShininess(float shininess)
{
    // Map Blinn-Phong exponent to roughness (approx)
    float rough = std::sqrt(std::max(0.0f, 2.0f / (shininess + 2.0f)));
    return std::clamp(rough, 0.02f, 1.0f);
}

static Material* EnsureMaterial(FbxSurfaceMaterial* fbxMat, MaterialList& materials, TextureFileList& texFiles, const std::string& baseDir)
{
    if (!fbxMat) return nullptr;
    const char* name = fbxMat->GetName();
    if (name) {
        if (Material* existing = materials.Find(name)) return existing;
    }

    auto* mtl = new MtlMicrofacet;
    if (name) mtl->SetName(name);

    auto findProp = [&](const char* n) -> FbxProperty { return fbxMat->FindProperty(n); };

    FbxProperty pBase = findProp("BaseColor");
    if (!pBase.IsValid()) pBase = fbxMat->FindProperty(FbxSurfaceMaterial::sDiffuse);
    Color baseColor = ReadColor(pBase, Color(0.7f));
    mtl->SetBaseColor(baseColor);
    if (TextureMap* tm = LoadTextureMap(pBase, texFiles, baseDir)) {
        mtl->SetBaseColorTexture(tm);
    }

    FbxProperty pMetal = findProp("Metalness");
    double metalVal = pMetal.IsValid() ? pMetal.Get<double>() : 0.0;
    mtl->SetMetallic(static_cast<float>(std::clamp(metalVal, 0.0, 1.0)));
    if (TextureMap* tm = LoadTextureMap(pMetal, texFiles, baseDir)) {
        mtl->SetMetallicTexture(tm);
    }

    FbxProperty pRough = findProp("Roughness");
    float roughVal = pRough.IsValid() ? static_cast<float>(std::clamp(pRough.Get<double>(), 0.0, 1.0)) : 1.0f;
    FbxProperty shininessProp = fbxMat->FindProperty(FbxSurfaceMaterial::sShininess);
    if (!pRough.IsValid() && shininessProp.IsValid()) {
        roughVal = RoughnessFromShininess(static_cast<float>(shininessProp.Get<double>()));
    }
    mtl->SetRoughness(roughVal);
    if (TextureMap* tm = LoadTextureMap(pRough, texFiles, baseDir)) {
        mtl->SetRoughnessTexture(tm);
    }

    FbxProperty pEmissive = fbxMat->FindProperty(FbxSurfaceMaterial::sEmissive);
    Color emissive = ReadColor(pEmissive, Color(0.0f));
    mtl->SetEmission(emissive);
    if (TextureMap* tm = LoadTextureMap(pEmissive, texFiles, baseDir)) {
        mtl->SetEmissionTexture(tm);
    }

    FbxProperty pOpacity = fbxMat->FindProperty(FbxSurfaceMaterial::sTransparencyFactor);
    if (!pOpacity.IsValid()) pOpacity = fbxMat->FindProperty("Opacity");
    double opacity = pOpacity.IsValid() ? pOpacity.Get<double>() : 1.0;
    opacity = std::clamp(opacity, 0.0, 1.0);
    Color trans = Color(1.0f - float(opacity));
    mtl->SetTransmittance(trans);
    if (TextureMap* tm = LoadTextureMap(pOpacity, texFiles, baseDir)) {
        mtl->SetTransmittanceTexture(tm);
    }

    FbxProperty pIor = fbxMat->FindProperty("RefractionIndex");
    if (pIor.IsValid()) {
        mtl->SetIOR(static_cast<float>(pIor.Get<double>()));
    }

    // Normal/Bump map
    FbxProperty pNormal = findProp("NormalMap");
    if (!pNormal.IsValid()) pNormal = fbxMat->FindProperty(FbxSurfaceMaterial::sBump);
    if (TextureMap* tm = LoadTextureMap(pNormal, texFiles, baseDir)) {
        mtl->SetBumpTexture(tm);
    }

    materials.push_back(mtl);
    return mtl;
}

static std::unique_ptr<TriObj> BuildMesh(FbxMesh* mesh)
{
    if (!mesh) return nullptr;
    mesh->RemoveBadPolygons();
    mesh->GenerateNormals(true, true, false);

    const int polyCount = mesh->GetPolygonCount();
    if (polyCount <= 0) return nullptr;

    const int vertPerPoly = mesh->GetPolygonSize(0);
    if (vertPerPoly != 3) return nullptr; // should be triangulated

    const int vertexCount = polyCount * 3;
    auto tri = std::make_unique<TriObj>();
    tri->SetNumVertex(vertexCount);
    tri->SetNumFaces(polyCount);

    const bool hasUV = mesh->GetElementUVCount() > 0;
    const bool hasNormals = mesh->GetElementNormalCount() > 0;
    if (hasUV) tri->SetNumTexVerts(vertexCount);
    if (hasNormals) tri->SetNumNormals(vertexCount);
    tri->SetNumMtls(1);

    FbxStringList uvSets;
    mesh->GetUVSetNames(uvSets);
    const char* uvSetName = (uvSets.GetCount() > 0) ? uvSets.GetStringAt(0) : nullptr;

    for (int p = 0; p < polyCount; ++p) {
        for (int v = 0; v < 3; ++v) {
            const int idx = p * 3 + v;
            const int ctrlIdx = mesh->GetPolygonVertex(p, v);
            FbxVector4 cp = mesh->GetControlPointAt(ctrlIdx);
            tri->V(idx) = Vec3f(float(cp[0]), float(cp[1]), float(cp[2]));
            tri->F(p).v[v] = idx;

            if (hasNormals) {
                FbxVector4 n;
                mesh->GetPolygonVertexNormal(p, v, n);
                tri->VN(idx) = Vec3f(float(n[0]), float(n[1]), float(n[2]));
                tri->FN(p).v[v] = idx;
            }

            if (hasUV && uvSetName) {
                FbxVector2 uv;
                bool unmapped = false;
                mesh->GetPolygonVertexUV(p, v, uvSetName, uv, unmapped);
                tri->VT(idx) = Vec3f(float(uv[0]), float(uv[1]), 0.0f);
                tri->FT(p).v[v] = idx;
            }
        }
    }

    // tri->mcfc[0] = polyCount; // single material covering all faces
    if (!hasNormals) tri->ComputeNormals();
    tri->ComputeBoundingBox();
    // Note: BVH building deferred until after all meshes are loaded (for parallel building)
    // tri->BuildAcceleration();
    return tri;
}

static void ProcessNode(FbxNode* fbxNode,
                        Node& parent,
                        ObjFileList& objList,
                        MaterialList& materials,
                        TextureFileList& texFiles,
                        const std::string& baseDir)
{
    if (!fbxNode) return;

    Node* node = new Node();
    parent.AppendChild(node);
    node->SetName(fbxNode->GetName());

    Matrix34f tm = ToMatrix34(fbxNode->EvaluateLocalTransform());
    node->Transform(tm);

    if (FbxMesh* mesh = fbxNode->GetMesh()) {
        auto tri = BuildMesh(mesh);
        if (tri) {
            TriObj* triPtr = tri.release();
            objList.push_back(triPtr);
            node->SetNodeObj(triPtr);
            if (Material* mtl = EnsureMaterial(fbxNode->GetMaterialCount() > 0 ? fbxNode->GetMaterial(0) : nullptr, materials, texFiles, baseDir)) {
                node->SetMaterial(mtl);
            }
        }
    }

    const int childCount = fbxNode->GetChildCount();
    for (int i = 0; i < childCount; ++i) {
        ProcessNode(fbxNode->GetChild(i), *node, objList, materials, texFiles, baseDir);
    }
}

static FbxNode* FindCameraNode(FbxNode* node, const char* desiredName)
{
    if (!node) return nullptr;
    if (node->GetCamera()) {
        if (!desiredName || std::strcmp(desiredName, node->GetName()) == 0) return node;
    }
    const int childCount = node->GetChildCount();
    for (int i = 0; i < childCount; ++i) {
        if (FbxNode* found = FindCameraNode(node->GetChild(i), desiredName)) return found;
    }
    return nullptr;
}

} // namespace

bool FBXLoader::IsSupported() { return true; }

bool FBXLoader::ImportScene(const char* filename, Node& parent, ObjFileList& objList, MaterialList& materials, TextureFileList& texFiles)
{
    if (!filename) return false;
    std::filesystem::path fbxPath(filename);
    std::string baseDir = fbxPath.has_parent_path() ? fbxPath.parent_path().string() : std::string();

    FBXScopedScene handle;
    if (!LoadSceneInternal(filename, handle)) {
        std::printf("ERROR: Failed to load FBX file \"%s\"\n", filename);
        return false;
    }

    ProcessNode(handle.scene->GetRootNode(), parent, objList, materials, texFiles, baseDir);
    return true;
}

bool FBXLoader::LoadCamera(const char* filename, const char* cameraName, Camera& cameraOut)
{
    if (!filename) return false;

    FBXScopedScene handle;
    if (!LoadSceneInternal(filename, handle)) {
        std::printf("ERROR: Failed to load FBX file \"%s\"\n", filename);
        return false;
    }

    FbxNode* camNode = FindCameraNode(handle.scene->GetRootNode(), cameraName);
    if (!camNode || !camNode->GetCamera()) {
        std::printf("ERROR: FBX camera \"%s\" not found in \"%s\"\n", cameraName ? cameraName : "(any)", filename);
        return false;
    }

    FbxCamera* fbxCam = camNode->GetCamera();
    cameraOut.ClearAnimation();

    std::vector<FbxTime> keyTimes;
    FbxAnimStack* stack = handle.scene->GetSrcObject<FbxAnimStack>(0);
    FbxAnimLayer* layer = stack ? stack->GetMember<FbxAnimLayer>(0) : nullptr;
    auto collectCurve = [&](FbxAnimCurve* c) {
        if (!c) return;
        const int kc = c->KeyGetCount();
        for (int i = 0; i < kc; ++i) keyTimes.push_back(c->KeyGetTime(i));
    };

    if (layer) {
        collectCurve(camNode->LclTranslation.GetCurve(layer, FBXSDK_CURVENODE_COMPONENT_X));
        collectCurve(camNode->LclTranslation.GetCurve(layer, FBXSDK_CURVENODE_COMPONENT_Y));
        collectCurve(camNode->LclTranslation.GetCurve(layer, FBXSDK_CURVENODE_COMPONENT_Z));
        collectCurve(camNode->LclRotation.GetCurve(layer, FBXSDK_CURVENODE_COMPONENT_X));
        collectCurve(camNode->LclRotation.GetCurve(layer, FBXSDK_CURVENODE_COMPONENT_Y));
        collectCurve(camNode->LclRotation.GetCurve(layer, FBXSDK_CURVENODE_COMPONENT_Z));
        collectCurve(fbxCam->FieldOfView.GetCurve(layer));
    }

    if (keyTimes.empty()) {
        FbxTime t0;
        t0.SetSecondDouble(0.0);
        keyTimes.push_back(t0);
    }

    std::sort(keyTimes.begin(), keyTimes.end(), [](const FbxTime& a, const FbxTime& b) { return a < b; });
    keyTimes.erase(std::unique(keyTimes.begin(), keyTimes.end(), [](const FbxTime& a, const FbxTime& b) { return a == b; }), keyTimes.end());

    for (const FbxTime& t : keyTimes) {
        FbxAMatrix global = camNode->EvaluateGlobalTransform(t);
        FbxVector4 pos = global.GetT();

        CameraKeyframe kf;
        kf.time = t.GetSecondDouble();
        kf.pos.Set(float(pos[0]), float(pos[1]), float(pos[2]));

        Vec3f forward = TransformDir(global, Vec3f(0, 0, -1)).GetNormalized();
        Vec3f up = TransformDir(global, Vec3f(0, 1, 0)).GetNormalized();
        if (forward.LengthSquared() == 0) forward.Set(0, 0, -1);
        if (up.LengthSquared() == 0) up.Set(0, 1, 0);

        kf.target = kf.pos + forward;
        kf.up = up;

        double fovY = fbxCam->FieldOfViewY.Get();
        if (fovY <= 0.0) fovY = fbxCam->FieldOfView.Get();
        kf.fov = fovY > 0.0 ? float(fovY) : 45.0f;
        kf.focaldist = (kf.target - kf.pos).Length();
        kf.dof = 0.0f;

        cameraOut.AddKeyframe(kf);
    }

    cameraOut.EvaluateAtTime(cameraOut.GetAnimationStart());
    return true;
}

#else  // RT_ENABLE_FBX

bool FBXLoader::IsSupported() { return false; }

bool FBXLoader::ImportScene(const char* filename, Node& parent, ObjFileList& objList, MaterialList& materials, TextureFileList& texFiles)
{
    (void)filename;
    (void)parent;
    (void)objList;
    (void)materials;
    (void)texFiles;
    std::printf("FBX support not enabled at build time; cannot import FBX scenes.\n");
    return false;
}

bool FBXLoader::LoadCamera(const char* filename, const char* cameraName, Camera& cameraOut)
{
    (void)filename;
    (void)cameraName;
    (void)cameraOut;
    std::printf("FBX support not enabled at build time; cannot import FBX cameras.\n");
    return false;
}

#endif // RT_ENABLE_FBX
