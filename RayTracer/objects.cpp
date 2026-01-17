#include "objects.h"
#include <algorithm>
#include <cmath>
#include "BVHConfig.h"

// Define M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace {
    constexpr float EPS = 1e-7f;
}

// Toggle BVH usage here as well so both translation units agree.
#ifndef RT_USE_MESH_BVH
#define RT_USE_MESH_BVH 1
#endif

// Sphere: unit sphere at origin in object space
bool Sphere::IntersectRay(Ray const& ray, HitInfo& hInfo, int hitSide) const {
    const Vec3f& p = ray.p;
    const Vec3f& d = ray.dir;

    const float A = d.Dot(d);
    const float B = 2.0f * p.Dot(d);
    const float C = p.Dot(p) - 1.0f;

    const float disc = B * B - 4 * A * C;
    if (disc < 0.0f) return false;

    const float s = sqrt(disc);
    const float inv2A = 0.5f / A;

    const float t0 = (-B - s) * inv2A;
    const float t1 = (-B + s) * inv2A;

    float t = -1.0f;
    if (t0 > EPS) {
        t = t0;
    }
    else if (t1 > EPS) {
        t = t1;
    }
    if (t < 0.0f || t >= hInfo.z) return false;

    const Vec3f hp = p + d * t;
    Vec3f GN = hp.GetNormalized();

    const bool front = (d.Dot(GN) < 0.0f);
    if ((hitSide == HIT_FRONT && !front) || (hitSide == HIT_BACK && front)) return false;

    hInfo.z     = t;
    hInfo.p     = hp;
    hInfo.GN    = GN;
    hInfo.N     = GN;      // sphere shading normal = geometry normal
    hInfo.front = front;

    // --- Spherical UVs, assuming Z is up (camera-forward setup) ---
    Vec3f n = hp.GetNormalized();

    // Longitude: rotation around Z axis, Latitude: from top pole (Z=+1)
    float phi = atan2f(n.y, n.x);                        // [-π, π]
    float theta = acosf(std::clamp(n.z, -1.0f, 1.0f));   // [0, π]

    // Map to [0,1]
    float u = (phi + float(M_PI)) / (2.0f * float(M_PI));  // east-west wrap
    float v = 1.0f - (theta / float(M_PI));                // top→bottom

    hInfo.uvw.Set(u, v, 0.0f);
    hInfo.duvw[0].Zero();
    hInfo.duvw[1].Zero();
    // Parametric derivatives on a unit sphere
    float sinTheta = std::sin(theta);
    float cosTheta = std::cos(theta);
    float sinPhi = std::sin(phi);
    float cosPhi = std::cos(phi);
    hInfo.dpdu = Vec3f(-sinTheta * sinPhi * 2.0f * float(M_PI),
                       sinTheta * cosPhi * 2.0f * float(M_PI),
                       0.0f);
    hInfo.dpdv = Vec3f(-float(M_PI) * cosTheta * cosPhi,
                       -float(M_PI) * cosTheta * sinPhi,
                       float(M_PI) * sinTheta);

    return true;
}

// Plane: finite square [-1,1]x[-1,1] at z=0, normal +Z in object space
bool Plane::IntersectRay(Ray const &ray, HitInfo &hInfo, int hitSide) const {
    const Vec3f N(0,0,1);
    const float denom = ray.dir.Dot(N);
    if (fabs(denom) < EPS) return false; // parallel

    const float t = -(ray.p.Dot(N)) / denom;
    if (t <= EPS || t >= hInfo.z) return false;

    const bool front = (denom < 0.0f);
    if ((hitSide == HIT_FRONT && !front) || (hitSide == HIT_BACK && front)) return false;

    const Vec3f hp = ray.p + ray.dir * t;

    // Finite square clip
    if (hp.x < -1.0f || hp.x >  1.0f) return false;
    if (hp.y < -1.0f || hp.y >  1.0f) return false;

    hInfo.z     = t;
    hInfo.p     = hp;
    hInfo.GN    = N;
    hInfo.N     = N;
    hInfo.front = front;

    // Map the finite square [-1,1]x[-1,1] -> [0,1]x[0,1]
    float u = 0.5f * (hp.x + 1.0f);
    float v = 0.5f * (hp.y + 1.0f);

    hInfo.uvw.x = u;
    hInfo.uvw.y = v;
    hInfo.uvw.z = 0.0f;

    hInfo.duvw[0].Zero();
    hInfo.duvw[1].Zero();
    hInfo.dpdu.Set(2.0f, 0.0f, 0.0f);
    hInfo.dpdv.Set(0.0f, 2.0f, 0.0f);

    return true;
}

void TriObj::BuildAcceleration()
{
    if (NF() == 0) return;
    if (!HasNormals()) {
        ComputeNormals();
    }
    ComputeBoundingBox();
#if RT_USE_MESH_BVH
    bvh.SetMesh(this, 4);
#endif
}

bool TriObj::IntersectTriangle(Ray const &ray, HitInfo &hInfo, int hitSide, unsigned int faceID) const {
    const auto& f  = F(faceID);
    const Vec3f v0 = V(f.v[0]);
    const Vec3f v1 = V(f.v[1]);
    const Vec3f v2 = V(f.v[2]);

    const Vec3f e1 = v1 - v0;
    const Vec3f e2 = v2 - v0;
    hInfo.dpdu.Zero();
    hInfo.dpdv.Zero();

    const Vec3f pvec = ray.dir ^ e2;           // cross
    const float det  = e1   .Dot(pvec);

    // Side control + backface/parallel reject
    if (hitSide == HIT_FRONT && det <= EPS) return false;
    if (hitSide == HIT_BACK  && det >= -EPS) return false;
    if (fabs(det) < EPS) return false;

    const float invDet = 1.0f / det;
    const Vec3f  tvec  = ray.p - v0;

    const float u = tvec.Dot(pvec) * invDet;
    if (u < 0.0f || u > 1.0f) return false;

    const Vec3f qvec = tvec ^ e1;
    const float v    = ray.dir.Dot(qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f) return false;

    const float t = e2.Dot(qvec) * invDet;
    if (t <= EPS || t >= hInfo.z) return false;

    const float w = 1.0f - u - v;

    // Geometry normal
    Vec3f GN = (e1 ^ e2);
    const float gnLen2 = GN.LengthSquared();
    if (gnLen2 > 0.0f) GN /= sqrt(gnLen2); else GN.Set(0,0,1);

    // Shading normal: interpolate vertex normals if present
    Vec3f N = GN;
    if (HasNormals()) {
        const auto& fn = FN(faceID);
        const Vec3f n0 = VN(fn.v[0]);
        const Vec3f n1 = VN(fn.v[1]);
        const Vec3f n2 = VN(fn.v[2]);
        N = (n0 * w + n1 * u + n2 * v);
        if (N.LengthSquared() > 0.0f) N.Normalize(); else N = GN;
    }

    const bool front = (ray.dir.Dot(GN) < 0.0f);
    if ((hitSide == HIT_FRONT && !front) || (hitSide == HIT_BACK && front)) return false;

    hInfo.z     = t;
    hInfo.p     = ray.p + ray.dir * t;
    hInfo.GN    = GN;
    hInfo.N     = N;
    hInfo.front = front;

    // --- Texture coordinates for triangle mesh ---
    const Vec3f bc(w, u, v);  // barycentric coordinates: w = 1 - u - v

    if (HasTextureVertices()) {
        Vec3f tex = GetTexCoord(faceID, bc);  // provided by cyTriMesh
        hInfo.uvw.x = tex.x;
        hInfo.uvw.y = tex.y;
        hInfo.uvw.z = 0.0f;
    }
    else {
        hInfo.uvw.Set(0.0f, 0.0f, 0.0f);
    }

    if (HasTextureVertices()) {
        const TriFace& ft = FT(faceID);
        const Vec3f uv0 = VT(ft.v[0]);
        const Vec3f uv1 = VT(ft.v[1]);
        const Vec3f uv2 = VT(ft.v[2]);
        const Vec3f duv1 = uv1 - uv0;
        const Vec3f duv2 = uv2 - uv0;
        const float detUV = duv1.x * duv2.y - duv1.y * duv2.x;
        if (std::fabs(detUV) > EPS) {
            const float invDetUV = 1.0f / detUV;
            hInfo.dpdu = (e1 * duv2.y - e2 * duv1.y) * invDetUV;
            hInfo.dpdv = (e2 * duv1.x - e1 * duv2.x) * invDetUV;
        }
    }

    if (hInfo.dpdu.LengthSquared() < EPS || hInfo.dpdv.LengthSquared() < EPS) {
        Vec3f tangent = (std::fabs(GN.x) > 0.5f) ? Vec3f(-GN.y, GN.x, 0.0f) : Vec3f(0.0f, -GN.z, GN.y);
        tangent.Normalize();
        Vec3f bitangent = GN ^ tangent;
        bitangent.Normalize();
        hInfo.dpdu = tangent;
        hInfo.dpdv = bitangent;
    }

    int matIndex = GetMaterialIndex(static_cast<int>(faceID));
    if (matIndex >= 0) hInfo.mtlID = matIndex;

    hInfo.duvw[0].Zero();
    hInfo.duvw[1].Zero();

    return true;
}

bool TriObj::IntersectRay(Ray const &ray, HitInfo &hInfo, int hitSide) const {
#if RT_USE_MESH_BVH
    if (g_rtUseMeshBVH) {
        if (NF() == 0) return false;

        const Box objBox(GetBoundMin(), GetBoundMax());
        if (!objBox.IsEmpty() && !objBox.IntersectRay(ray, hInfo.z)) return false;

        return TraceBVHNode(ray, hInfo, hitSide, bvh.GetRootNodeID());
    }
#endif
    // Brute-force fallback
    bool anyHit = false;
    for (unsigned int i = 0; i < NF(); ++i) {
        HitInfo tmp = hInfo; // carry current upper bound
        if (IntersectTriangle(ray, tmp, hitSide, i)) {
            hInfo   = tmp;
            anyHit  = true;
        }
    }
    return anyHit;
}
