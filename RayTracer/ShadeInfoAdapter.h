#pragma once

#include "ShadingConfig.h"
#include "renderer.h"
#include "scene.h"
#include "IntersectNodeAdapter.h"
#include "BumpMapping.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class PhotonMap; // Forward declaration
class PhotonMapBuilder; // Forward declaration

class ShadeInfoAdapter : public ShadeInfo {
public:
    static const TexturedColor& DefaultEnvironment() { static TexturedColor defEnv; return defEnv; }

    ShadeInfoAdapter(const std::vector<Light*>& lightList, const Scene* scenePtr, RNG& r)
        : ShadeInfo(lightList, scenePtr ? scenePtr->environment : DefaultEnvironment(), r), scene(scenePtr) {
    }

    // Photon map access
    void SetPhotonMap(const PhotonMap* pmap) { photonMap = pmap; }
    void SetCausticsMap(const PhotonMap* cmap) { causticsMap = cmap; }
    const PhotonMap* GetPhotonMap() const { return photonMap; }
    const PhotonMap* GetCausticsMap() const { return causticsMap; }

    // Photon search parameters with defaults
    float GetPhotonSearchRadius() const { return photonSearchRadius; }
    float GetCausticSearchRadius() const { return causticSearchRadius; }
    int GetPhotonSearchCount() const { return photonSearchCount; }

    void SetPhotonSearchRadius(float r) { photonSearchRadius = r; }
    void SetCausticSearchRadius(float r) { causticSearchRadius = r; }
    void SetPhotonSearchCount(int c) { photonSearchCount = c; }

    int  GetDiffuseDepth() const { return diffuseDepth; }
    void SetDiffuseDepth(int d) { diffuseDepth = d; }

    // Allow a few specular bounces for reflection/refraction
    bool CanBounce() const override { return bounce < kMaxSpecularBounces; }

    bool IsFirstDiffuseBounce() const { return diffuseDepth <= 1; }
    bool IsSecondOrLaterBounce() const { return bounce >= 1; }

    float TraceShadowRay(Ray const& r, float t_max = BIGFLOAT) const override {
        if (!scene) return 1.0f;

        const float Llen = r.dir.Length();
        if (Llen < 1e-12f) return 1.0f;

        const bool  finiteSegment = (t_max < BIGFLOAT * 0.5f);
        const float segLenWorld = finiteSegment ? (Llen * t_max) : BIGFLOAT;
        const Vec3f Ldir = r.dir / Llen;

        // Robust bias: normal-based + a small component along the shadow direction
        const Vec3f N = this->N();
        const float nDotL = N % Ldir;
        const float baseBias = 1e-4f;
        const float nBias = baseBias * (1.0f + 2.0f * (1.0f - std::abs(nDotL))); // a bit larger at grazing
        const float dBias = baseBias * 0.5f; // tiny push along the ray to avoid t=0 misses at contacts

        Ray sRay;
        sRay.p = P() + ((nDotL >= 0.0f) ? N : -N) * nBias + Ldir * dBias;
        sRay.dir = Ldir; // normalized

        HitInfo h; h.Init(); h.z = BIGFLOAT;
        if (IntersectNode(&scene->rootNode, sRay, h, HIT_FRONT_AND_BACK)) {
            const float dist = (h.p - sRay.p).Length();

            // Ignore extremely near self-intersections on the same node (receiver)
            if (h.node == GetNode() && dist < (nBias + dBias) * 6.0f) return 1.0f; // More conservative for interior

            if (!finiteSegment) {
                // Directional: any positive hit occludes
                return dist > 0.0f ? 0.0f : 1.0f;
            }
            else {
                // Point/spot: occlude only if blocker is before the light
                const float allowance = nBias * 1.5f;
                return (dist < segLenWorld - allowance) ? 0.0f : 1.0f;
            }
        }
        return 1.0f;
    }

    float TraceShadowRay(Vec3f const& dir, float t_max = BIGFLOAT) const override {
        return TraceShadowRay(Ray(P(), dir), t_max);
    }

    Color TraceSecondaryRay(Ray const& inRay, float& dist, bool recursive = true, bool isReflection = true) const {
        dist = BIGFLOAT;
        if (!scene) return Color(0, 0, 0);
        if (!CanBounce()) return Color(0, 0, 0);

        const float baseBias = 1e-4f;
        Vec3f dir = inRay.dir;
        float dlen = dir.Length();
        if (dlen < 1e-20f) return Color(0, 0, 0);
        dir /= dlen;

        const Vec3f N = this->N();
        const float nDotD = N % dir;
        const float nBias = baseBias * (1.0f + 2.0f * (1.0f - std::abs(nDotD))); // slightly larger at grazing
        const float dBias = baseBias * 0.5f;

        Ray ray;
        ray.p = P() + ((nDotD >= 0.0f) ? N : -N) * nBias + dir * dBias;
        ray.dir = dir;

        HitInfo h;
        if (!isReflection) {
            // Check for light intersections using the same biased ray
            HitInfo lightHit;
            float bestLightT = BIGFLOAT;
            const Light* visibleLight = nullptr;
            for (size_t li = 0; li < scene->lights.size(); ++li) {
                const Light* L = scene->lights[li];
                if (!L->IsRenderable()) continue;
                HitInfo h; h.Init();
                if (L->IntersectRay(ray, h, HIT_FRONT_AND_BACK)) {
                    if (h.z > 0.0f && h.z < bestLightT) {
                        bestLightT = h.z;
                        lightHit = h;
                        visibleLight = L;
                    }
                }
            }

            h.Init(); h.z = BIGFLOAT;
            if (IntersectNode(&scene->rootNode, ray, h, HIT_FRONT_AND_BACK)) {
                // Check if a light is closer
                if (visibleLight && bestLightT < h.z) {
                    // Use the light hit
                    ShadeInfoAdapter s2(lights, scene, rng);
                    s2.SetPixel(X(), Y());
                    s2.SetHit(ray, lightHit);
                    CopyPhotonContext(s2);
                    dist = bestLightT;
                    Color rad = visibleLight->Radiance(s2);
                    rad.Clamp(0.0f, 1.0f);
                    return rad;
                }
            }
            else if (visibleLight) {
                // Only light hit
                ShadeInfoAdapter s2(lights, scene, rng);
                s2.SetPixel(X(), Y());
                s2.SetHit(ray, lightHit);
                CopyPhotonContext(s2);
                dist = bestLightT;
                Color rad = visibleLight->Radiance(s2);
                rad.Clamp(0.0f, 1.0f);
                return rad;
            }
        }
        else {
            // Reflection/refraction paths should also consider renderable lights
            HitInfo lightHit; float bestLightT = BIGFLOAT; const Light* visibleLight = nullptr;
            for (size_t li = 0; li < scene->lights.size(); ++li) {
                const Light* L = scene->lights[li];
                if (!L->IsRenderable()) continue;
                HitInfo hL; hL.Init();
                if (L->IntersectRay(ray, hL, HIT_FRONT_AND_BACK)) {
                    if (hL.z > 0.0f && hL.z < bestLightT) { bestLightT = hL.z; lightHit = hL; visibleLight = L; }
                }
            }
            h.Init(); h.z = BIGFLOAT;
            bool hitGeom = IntersectNode(&scene->rootNode, ray, h, HIT_FRONT_AND_BACK);
            if (visibleLight && (!hitGeom || bestLightT < h.z)) {
                ShadeInfoAdapter s2(lights, scene, rng);
                s2.SetPixel(X(), Y());
                s2.SetHit(ray, lightHit);
                CopyPhotonContext(s2);
                dist = bestLightT;
                Color rad = visibleLight->Radiance(s2);
                rad.Clamp(0.0f, 1.0f);
                return rad;
            }
        }

        if (h.z < BIGFLOAT) {
            const float hitDist = (h.p - ray.p).Length();
            dist = h.front ? hitDist : 0.0f;

            // Check if we hit a light source
            if (h.light && h.node) {
                const Object* obj = h.node->GetNodeObj();
                const Light* light = dynamic_cast<const Light*>(obj);
                if (light) {
                    ShadeInfoAdapter s2(lights, scene, rng);
                    s2.SetPixel(X(), Y());
                    s2.SetHit(ray, h);
                    CopyPhotonContext(s2);
                    s2.SetDiffuseDepth(this->diffuseDepth);
                    Color rad = light->Radiance(s2);
                    rad.Clamp(0.0f, 1.0f);
                    return rad;
                }
                // If cast failed, fall through to material handling
            }

            const Material* mtl = h.node ? h.node->GetMaterial() : nullptr;
            if (!mtl) return Color(0, 0, 0);

            ApplyBumpMapping(mtl, h);

            ShadeInfoAdapter s2(lights, scene, rng);
            s2.SetPixel(X(), Y());
            s2.SetHit(ray, h);
            CopyPhotonContext(s2);
            s2.SetDiffuseDepth(this->diffuseDepth + (mtl->IsPhotonSurface(h.mtlID) ? 1 : 0));

            if (recursive) {
                s2.bounce = this->bounce + 1;
            }
            else {
                // Force next Shade() to think it's at max depth => no further secondary rays.
                s2.bounce = kMaxSpecularBounces;       // single-hit shading only
            }

            return mtl->Shade(s2);
        }
        else {
            if (scene) return scene->environment.EvalEnvironment(ray.dir.GetNormalized());
            return Color(0, 0, 0);
        }
    }

    Color TraceSecondaryRay(Vec3f const& dir, float& dist, bool isReflection = true) const {
        return TraceSecondaryRay(Ray(P(), dir), dist, true, isReflection);
    }

private:
    const Scene* scene = nullptr;
    const PhotonMap* photonMap = nullptr;
    const PhotonMap* causticsMap = nullptr;
    float photonSearchRadius = 0.2f;
    float causticSearchRadius = 0.05f;
    int photonSearchCount = 100;
    int diffuseDepth = 0;
    static constexpr int kMaxSpecularBounces = 5;

    void CopyPhotonContext(ShadeInfoAdapter& dst) const {
        dst.SetPhotonMap(photonMap);
        dst.SetCausticsMap(causticsMap);
        dst.SetPhotonSearchRadius(photonSearchRadius);
        dst.SetCausticSearchRadius(causticSearchRadius);
        dst.SetPhotonSearchCount(photonSearchCount);
        dst.SetDiffuseDepth(diffuseDepth);
    }
};
