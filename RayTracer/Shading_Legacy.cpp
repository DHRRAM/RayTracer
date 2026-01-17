#include "ShadingConfig.h"

#if SHADING_USE_LEGACY

#include <algorithm>
#include <cmath>
#include "materials.h"
#include "lights.h"
#include "renderer.h"
#include "scene.h"
#include "ShadeInfoAdapter.h"
#include "rng.h"
#include "IntersectNodeAdapter.h"
#include "photonmap.h"
#include "PhotonMappingConfig.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const bool USE_COSINE_WEIGHTED = true;  // Set to false for uniform hemisphere sampling comparison

// Photon mapping flags now come from PhotonMappingConfig.h
using PhotonMappingConfig::USE_PHOTON_MAP_DIRECT;
using PhotonMappingConfig::USE_PHOTON_MAP_INDIRECT;

using std::max;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Helper function to get irradiance from photon map
static Color GetPhotonMapIrradiance(ShadeInfo const& sInfo, const PhotonMap* pmap, float radius, int maxPhotons) {
    if (!pmap || pmap->NumPhotons() == 0) return Color(0, 0, 0);

    Color irrad;
    Vec3f avgDir;

    // Use the EstimateIrradiance template method with the surface normal
    // Template parameters: <maxPhotons, filterType>
    if (maxPhotons <= 30) {
        pmap->EstimateIrradiance<30, PHOTONMAP_FILTER_LINEAR>(irrad, avgDir, radius, sInfo.P(), sInfo.N());
    }
    else if (maxPhotons <= 50) {
        pmap->EstimateIrradiance<50, PHOTONMAP_FILTER_LINEAR>(irrad, avgDir, radius, sInfo.P(), sInfo.N());
    }
    else {
        pmap->EstimateIrradiance<100, PHOTONMAP_FILTER_LINEAR>(irrad, avgDir, radius, sInfo.P(), sInfo.N());
    }

    return irrad;
}

// Map (u1,u2) in [0,1)^2 to a concentric unit disk (Shirley-Chiu)
static inline void SampleConcentricDisk(float u1, float u2, float& dx, float& dy) {
    float sx = 2.0f * u1 - 1.0f;
    float sy = 2.0f * u2 - 1.0f;
    if (sx == 0.0f && sy == 0.0f) { dx = dy = 0.0f; return; }

    const float PI_4 = 0.78539816339f; // π/4
    const float PI_2 = 1.57079632679f; // π/2

    float r, phi;
    if (fabsf(sx) > fabsf(sy)) {
        r = sx;
        phi = PI_4 * (sy / sx);
    }
    else {
        r = sy;
        phi = PI_2 - PI_4 * (sx / sy);
    }
    dx = r * cosf(phi);
    dy = r * sinf(phi);
}

// Build an ONB (u,v) perpendicular to n
static inline void BuildONB(const cy::Vec3f& n, cy::Vec3f& u, cy::Vec3f& v) {
    if (fabsf(n.x) > 0.5f) u = cy::Vec3f(0, 1, 0) ^ n;
    else                   u = cy::Vec3f(1, 0, 0) ^ n;
    u.Normalize();
    v = n ^ u;
    v.Normalize();
}

static inline Vec3f SafeNormalize(Vec3f v) {
    float len2 = v.LengthSquared();
    if (len2 <= 0.0f) return Vec3f(0, 0, 0);
    return v / sqrtf(len2);
}

bool PointLight::GenerateSample(SamplerInfo const& sInfo, Vec3f& dir, DirSampler::Info& si) const {
    // Legacy shading does not use light sampling; keep stub to satisfy linkage.
    si.SetVoid();
    (void)sInfo; (void)dir;
    return false;
}

void PointLight::GetSampleInfo(SamplerInfo const& sInfo, Vec3f const& dir, DirSampler::Info& si) const {
    si.SetVoid();
    (void)sInfo; (void)dir;
}

// Legacy stubs for BSDF evaluation (only needed to satisfy link when legacy mode is active)
void MtlPhong::GetSampleInfo(SamplerInfo const& /*sInfo*/, Vec3f const& /*dir*/, DirSampler::Info& si) const {
    si.SetVoid();
}

void MtlBlinn::GetSampleInfo(SamplerInfo const& /*sInfo*/, Vec3f const& /*dir*/, DirSampler::Info& si) const {
    si.SetVoid();
}

void MtlMicrofacet::GetSampleInfo(SamplerInfo const& /*sInfo*/, Vec3f const& /*dir*/, DirSampler::Info& si) const {
    si.SetVoid();
}

Color PointLight::Illuminate(ShadeInfo const& sInfo, cy::Vec3f& dir) const {
    // Hard shadow fallback for zero-size (classic point light)
    cy::Vec3f P = sInfo.P();
    cy::Vec3f toC = position - P;                      // to center
    float distC = toC.Length();
    if (distC < 1e-20f) { dir = cy::Vec3f(0, 0, 0); return Color(0, 0, 0); }
    cy::Vec3f L = toC / distC;                         // main light direction
    dir = L;

    // Inverse square attenuation
    float atten = 1.0f;
    if (attenuation > 0.0f) {
        atten = 1.0f / ((attenuation * distC) * (attenuation * distC));
    }

    if (size <= 0.0f) {
        // segment shadow ray to the light center
        return intensity * sInfo.TraceShadowRay(Ray(P, toC), 1.0f) * atten;
    }

    // --- Area light (spherical) soft shadow sampling on the projected disk ---

    // Disk basis
    cy::Vec3f U, V;
    BuildONB(L, U, V);

    // More aggressive adaptive sampling settings for better performance
    constexpr int kMinSamples = 1;
    constexpr int kMaxSamples = 4;

    // Per-pixel-sample random shift (Cranley–Patterson) to decorrelate
    const float jx = sInfo.RandomFloat();
    const float jy = sInfo.RandomFloat();

    int n = 0;
    int lit = 0;
    int dark = 0;

    auto sampleOnce = [&](int sIdx)
        {
            // Halton (base 2,3) with jitter/shift (wrap to [0,1))
            float h2 = Halton(sIdx + 1, 2); h2 = h2 + jx; h2 -= floorf(h2);
            float h3 = Halton(sIdx + 1, 3); h3 = h3 + jy; h3 -= floorf(h3);

            // --- Sample within cone subtending the spherical light ---
           // Half-angle of visible cone
            float gamma = asinf(size / distC);

            // Uniform solid-angle sampling of that cone
            float u1 = h2; float u2 = h3;
            float cosTheta = 1.0f - u1 * (1.0f - cosf(gamma));
            float sinTheta = sqrtf(max(0.0f, 1.0f - cosTheta * cosTheta));
            float phi = 2.0f * float(M_PI) * u2;

            // Local basis around main direction L
            cy::Vec3f sampleDir = (U * cosf(phi) * sinTheta +
                V * sinf(phi) * sinTheta +
                L * cosTheta).GetNormalized();

            // Trace the shadow ray in that direction, limited to the light distance
            float vis = sInfo.TraceShadowRay(Ray(P, sampleDir * distC), 1.0f);

            ++n;
            if (vis >= 0.999f) ++lit;
            else if (vis <= 0.001f) ++dark;

            return vis;
        };

    // First do the minimal set
    float sumVis = 0.0f;
    for (int i = 0; i < kMinSamples; ++i) sumVis += sampleOnce(i);

    // Early exit if unanimous (all lit or all dark) - much more aggressive
    if (lit == kMinSamples || dark == kMinSamples) {
        float avgVis = sumVis / float(n);
        return intensity * avgVis * atten;
    }

    // If not unanimous, take remaining samples
    for (int i = kMinSamples; i < kMaxSamples; ++i) {
        sumVis += sampleOnce(i);

        // Early termination if we reach consensus (>75% agreement)
        if (i >= kMinSamples + 2) {  // After at least 6 total samples
            if (lit >= n * 0.75f || dark >= n * 0.75f) break;
        }
    }

    float avgVis = sumVis / float(n);
    if (avgVis < 0.0f) avgVis = 0.0f;
    if (avgVis > 1.0f) avgVis = 1.0f;

    // Return attenuated intensity with inverse square attenuation
    return intensity * avgVis * atten;
}

bool PointLight::IntersectRay(Ray const& ray, HitInfo& hInfo, int hitSide) const {
    if (size <= 0.0f) return false; // point light doesn't intersect

    Vec3f toC = position - ray.p;
    float distC = toC.Length();
    float tca = toC % ray.dir;
    if (tca < 0) return false;
    float d2 = toC.LengthSquared() - tca * tca;
    if (d2 > size * size) return false;
    float thc = sqrtf(size * size - d2);
    float t0 = tca - thc;
    float t1 = tca + thc;
    if (t0 > t1) std::swap(t0, t1);
    if (t0 < 0) t0 = t1;
    if (t0 < 0) return false;

    hInfo.z = t0;
    hInfo.p = ray.p + ray.dir * t0;
    hInfo.N = (hInfo.p - position).GetNormalized();
    hInfo.GN = hInfo.N;
    hInfo.mtlID = 0;
    hInfo.light = true;
    return true;
}

void PointLight::RandomPhoton(RNG& rng, Ray& r, Color& c) const {
    // Generate a random position on the sphere (if size > 0) or from the point
    Vec3f photonPos = position;
    if (size > 0.0f) {
        // Uniform sampling on sphere surface
        float u1 = rng.RandomFloat();
        float u2 = rng.RandomFloat();

        float z = 1.0f - 2.0f * u1;
        float r_val = sqrtf(std::max(0.0f, 1.0f - z * z));
        float phi = 2.0f * float(M_PI) * u2;

        Vec3f offset(r_val * cosf(phi), r_val * sinf(phi), z);
        photonPos = position + offset * size;
    }

    // Generate a random direction
    // For a spherical/point light, we emit in all directions uniformly
    float u1 = rng.RandomFloat();
    float u2 = rng.RandomFloat();

    float z = 1.0f - 2.0f * u1;
    float r_val = sqrtf(std::max(0.0f, 1.0f - z * z));
    float phi = 2.0f * float(M_PI) * u2;

    Vec3f dir(r_val * cosf(phi), r_val * sinf(phi), z);
    dir.Normalize();
    
    // If we have a spherical light, make sure the direction points outward
    if (size > 0.0f) {
        Vec3f normal = (photonPos - position).GetNormalized();
        // Use hemisphere sampling around the normal
        Vec3f U, V;
        if (fabsf(normal.x) > 0.5f) {
            U = (Vec3f(0, 1, 0) ^ normal).GetNormalized();
        }
        else {
            U = (Vec3f(1, 0, 0) ^ normal).GetNormalized();
        }
        V = (normal ^ U).GetNormalized();

        // Cosine-weighted hemisphere sampling
        u1 = rng.RandomFloat();
        u2 = rng.RandomFloat();
        float cosTheta = sqrtf(u1);
        float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
        phi = 2.0f * float(M_PI) * u2;

        dir = (U * (sinTheta * cosf(phi)) +
            V * (sinTheta * sinf(phi)) +
            normal * cosTheta).GetNormalized();
    }
    
    r.p = photonPos;
    r.dir = dir;
    
    c = intensity * 4.0f * M_PI;
}

static float FresnelExact(const Vec3f& I, const Vec3f& N_in, float ior, bool frontFace) {
    Vec3f N = N_in;
    float cosi = clampf(I % N, -1.0f, 1.0f);
    float etai = 1.0f, etat = ior;

    // Ensure the normal always points against the incident ray
    if (cosi > 0.0f) {
        std::swap(etai, etat);
        N = -N;
    }
    cosi = fabs(cosi);

    // Snell's law
    float sint = etai / etat * sqrtf(max(0.0f, 1.0f - cosi * cosi));
    if (sint >= 1.0f) {
        return 1.0f; // total internal reflection
    }

    float cost = sqrtf(max(0.0f, 1.0f - sint * sint));
    float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
    float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
    return clampf(0.5f * (Rs * Rs + Rp * Rp), 0.0f, 1.0f);
}

// Apply per-channel Beer-Lambert absorption to a color given absorption coefficients and distance.
static inline Color ApplyAbsorption(const Color& c, const Color& absorption, float dist) {
    if (dist <= 0.0f) return c;
    // If absorption is effectively zero, return original color to preserve current appearance
    if (absorption.r == 0.0f && absorption.g == 0.0f && absorption.b == 0.0f) return c;
    const float ar = expf(-absorption.r * dist);
    const float ag = expf(-absorption.g * dist);
    const float ab = expf(-absorption.b * dist);
    return Color(c.r * ar, c.g * ag, c.b * ab);
}

// Glossy sampling for reflection or refraction
static Color SampleGlossy(const Vec3f& axis, float alpha, bool isReflection, const Vec3f& V, ShadeInfo const& sInfo, int glossySamples, const Color& absorption = Color(0, 0, 0)) {
    Color result(0.0f);
    Vec3f U, Vb;
    BuildONB(axis, U, Vb);

    for (int s = 0; s < glossySamples; ++s) {
        float u1 = sInfo.RandomFloat();
        float u2 = sInfo.RandomFloat();

        float cosTheta = powf(u1, 1.0f / (alpha + 1.0f));
        float sinTheta = sqrtf(std::max(0.0f, 1.0f - cosTheta * cosTheta));
        float phi = 2.0f * float(M_PI) * u2;
        Vec3f H = (U * cosf(phi) * sinTheta + Vb * sinf(phi) * sinTheta + axis * cosTheta).GetNormalized();

        Vec3f dir;
        if (isReflection) {
            // GGX half-vector sampling for reflection
            dir = 2.0f * (V % H) * H - V;  // Reflect about H
        }
        else {
            // Cone sampling for refraction
            dir = H;
        }
        dir.Normalize();

        float dist;
        Color c = sInfo.TraceSecondaryRay(dir, dist, isReflection);
        if (!isReflection) c = ApplyAbsorption(c, absorption, dist);  // Only for refraction
        result += c;
    }
    return result / float(glossySamples);
}

// Blinn Shading
Color MtlBlinn::Shade(ShadeInfo const& sInfo) const {
    Vec3f N = sInfo.N().GetNormalized();
    if (!sInfo.IsFront()) N = -N;  // Flip normal for backface hits to ensure correct shading

    const Vec3f V = sInfo.V().GetNormalized();
    const Vec3f I = -V;

    // Local illumination (direct)
    Color local(0.0f);
    float alpha = sInfo.Eval(Glossiness());

    // If photon-map direct is enabled, gather irradiance from photon map and use it
    const ShadeInfoAdapter* sInfoAd = dynamic_cast<const ShadeInfoAdapter*>(&sInfo);
    const PhotonMap* pmap = (sInfoAd && USE_PHOTON_MAP_DIRECT) ? sInfoAd->GetPhotonMap() : nullptr;

    if (pmap && pmap->NumPhotons() > 0) {
        Color irrad = GetPhotonMapIrradiance(sInfo, pmap, sInfoAd->GetPhotonSearchRadius(), sInfoAd->GetPhotonSearchCount());
        // photon map stores irradiance (energy) — multiply by diffuse reflectance / pi to get outgoing radiance
        local += (sInfo.Eval(Diffuse()) / Pi<float>()) * irrad;

        // Still compute specular highlights directly from lights
        for (int i = 0; i < sInfo.NumLights(); ++i) {
            Vec3f Ldir;
            const Light* L = sInfo.GetLight(i);
            const Color Li = L->Illuminate(sInfo, Ldir);
            if (L->IsAmbient()) continue;
            Ldir.Normalize();
            const float NdotL = max(0.0f, N % Ldir);
            if (NdotL <= 0.0f) continue;
            // Only add specular component
            const Vec3f H = (Ldir + V).GetNormalized();
            const float NdotH = max(0.0f, N % H);
            if (NdotH > 0.0f)
                local += (sInfo.Eval(Specular()) * (alpha + 2.0f) / (8.0f * Pi<float>())) * Li * pow(NdotH, alpha);
        }
    }
    else {
        for (int i = 0; i < sInfo.NumLights(); ++i) {
            Vec3f Ldir;
            const Light* L = sInfo.GetLight(i);
            const Color Li = L->Illuminate(sInfo, Ldir);
            if (L->IsAmbient()) { local += (sInfo.Eval(Diffuse()) / Pi<float>()) * Li; continue; }
            Ldir.Normalize();
            const float NdotL = max(0.0f, N % Ldir);
            if (NdotL <= 0.0f) continue;
            local += (sInfo.Eval(Diffuse()) / Pi<float>()) * Li * NdotL;
            const Vec3f H = (Ldir + V).GetNormalized();
            const float NdotH = max(0.0f, N % H);
            if (NdotH > 0.0f)
                local += (sInfo.Eval(Specular()) * (alpha + 2.0f) / (8.0f * Pi<float>())) * Li * pow(NdotH, alpha);
        }
    }

    // Indirect illumination
    Color indirect(0.0f);
    Color causticTerm(0.0f);
    const PhotonMap* indirectPmap = nullptr;
    const PhotonMap* causticsPmap = nullptr;
    bool useIndirectMap = false;
    if (sInfoAd && USE_PHOTON_MAP_INDIRECT) {
        indirectPmap = sInfoAd->GetPhotonMap();
        causticsPmap = sInfoAd->GetCausticsMap();
        useIndirectMap = (indirectPmap && indirectPmap->NumPhotons() > 0);
    }

    const bool hasDiffuse = (sInfo.Eval(Diffuse()).Max() > 1e-6f);

    // Always allow caustics map on diffuse surfaces so caustics remain visible through specular transport
    if (hasDiffuse && sInfoAd && causticsPmap && causticsPmap->NumPhotons() > 0) {
        Color causticsIrrad = GetPhotonMapIrradiance(sInfo, causticsPmap, sInfoAd->GetCausticSearchRadius(), sInfoAd->GetPhotonSearchCount());
        causticTerm = (sInfo.Eval(Diffuse()) / Pi<float>()) * causticsIrrad;
    }

    const bool canDiffuseBounce = hasDiffuse && sInfo.CanBounce();
    if (sInfoAd && useIndirectMap && !sInfoAd->IsFirstDiffuseBounce()) {
        // Indirect (non-caustic) photon map only on secondary diffuse bounces
        Color indirectIrrad = GetPhotonMapIrradiance(sInfo, indirectPmap, sInfoAd->GetPhotonSearchRadius(), sInfoAd->GetPhotonSearchCount());
        indirect += (sInfo.Eval(Diffuse()) / Pi<float>()) * indirectIrrad;
    }
    else if (canDiffuseBounce) {
        Vec3f indirDir;
        DirSampler::Info si;
        if (GenerateSample(sInfo, indirDir, si)) {
            float dist;
            Color Li = sInfo.TraceSecondaryRay(indirDir, dist, false);
            indirect = Li * si.mult / si.prob;
        }
    }

    indirect += causticTerm;

    local += indirect;

    const Color refl = sInfo.Eval(Reflection());
    const Color refrRaw = sInfo.Eval(Refraction());
    const float ior = IOR();

    const bool isDielectric = (refrRaw.Max() > 1e-6f);
    Color refrTint = isDielectric ? (refrRaw.Max() > 1e-6f ? refrRaw : sInfo.Eval(Diffuse())) : Color(0, 0, 0);

    Color result = local;

    if (!USE_PHOTON_MAP_DIRECT || USE_PHOTON_MAP_INDIRECT) {
        if ((isDielectric || refl.Max() > 1e-4f) && sInfo.CanBounce()) {
            const float kr = isDielectric ? FresnelExact(I, N, ior, sInfo.IsFront()) : 1.0f;

            // Reflection (adaptive samples: perfect/mild gloss = fewer)
            const float alphaVal = sInfo.Eval(Glossiness());
            const int glossySamples = 1;
            Color reflectionColor = SampleGlossy(N, alphaVal, true, V, sInfo, glossySamples, Color(0, 0, 0));

            // Refraction
            Color refractionColor(0.0f);
            if (isDielectric) {
                float n1 = 1.0f, n2 = ior;
                Vec3f NN = N;
                float cosi = -(NN % I);
                if (!sInfo.IsFront()) { std::swap(n1, n2); cosi = -(NN % I); }
                const float eta = n1 / n2;
                const float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
                if (k >= 0.0f) {
                    Vec3f T = eta * I + (eta * cosi - sqrtf(k)) * NN;
                    T.Normalize();

                    const int glossySamples = 1;

                    Vec3f U, Vb;
                    BuildONB(T, U, Vb);

                    refractionColor = SampleGlossy(T, sInfo.Eval(Glossiness()), false, V, sInfo, glossySamples, Absorption());
                }
            }

            if (isDielectric) {
                const Color oneMinusRefl = Color(1.0f) - refl;
                result = local * oneMinusRefl
                    + reflectionColor * kr
                    + refractionColor * ((1.0f - kr) * refrTint);
            }
            else {
                result = (local + reflectionColor * refl);
            }
        }
    }
    Color emission = sInfo.Eval(Emission());
    result += emission;

    return result;
}

// Phong Shading
Color MtlPhong::Shade(ShadeInfo const& sInfo) const {
    Vec3f N = sInfo.N().GetNormalized();
    if (!sInfo.IsFront()) N = -N;  // Flip normal for backface hits to ensure correct shading

    const Vec3f V = sInfo.V().GetNormalized();
    const Vec3f I = -V;

    // Local illumination (direct)
    Color local(0.0f);
    float alpha = sInfo.Eval(Glossiness());

    const ShadeInfoAdapter* sInfoAd2 = dynamic_cast<const ShadeInfoAdapter*>(&sInfo);
    const PhotonMap* pmapPh = (sInfoAd2 && USE_PHOTON_MAP_DIRECT) ? sInfoAd2->GetPhotonMap() : nullptr;
    if (pmapPh && pmapPh->NumPhotons() > 0) {
        Color irrad = GetPhotonMapIrradiance(sInfo, pmapPh, sInfoAd2->GetPhotonSearchRadius(), sInfoAd2->GetPhotonSearchCount());
        local += (sInfo.Eval(Diffuse()) / Pi<float>()) * irrad;

        // Still compute specular highlights directly from lights
        for (int i = 0; i < sInfo.NumLights(); ++i) {
            Vec3f Ldir;
            const Light* L = sInfo.GetLight(i);
            Color Li = L->Illuminate(sInfo, Ldir);
            if (L->IsAmbient()) continue;
            Ldir.Normalize();
            const float NdotL = max(0.0f, N % Ldir);
            if (NdotL <= 0.0f) continue;
            // Only add specular component
            const Vec3f R = (2.0f * (N % Ldir) * N - Ldir).GetNormalized();
            const float RdotV = max(0.0f, R % V);
            if (RdotV > 0.0f)
                local += (sInfo.Eval(Specular()) * (alpha + 1.0f) / (2.0f * Pi<float>())) * Li * pow(RdotV, alpha);
        }
    }
    else {
        for (int i = 0; i < sInfo.NumLights(); ++i) {
            Vec3f Ldir;
            const Light* L = sInfo.GetLight(i);
            Color Li = L->Illuminate(sInfo, Ldir);
            if (L->IsAmbient()) { local += (sInfo.Eval(Diffuse()) / Pi<float>()) * Li; continue; }
            Ldir.Normalize();
            const float NdotL = max(0.0f, N % Ldir);
            if (NdotL <= 0.0f) continue;
            Color lightContrib = (sInfo.Eval(Diffuse()) / Pi<float>()) * Li * NdotL;
            const Vec3f R = (2.0f * (N % Ldir) * N - Ldir).GetNormalized();
            const float RdotV = max(0.0f, R % V);
            if (RdotV > 0.0f)
                lightContrib += (sInfo.Eval(Specular()) * (alpha + 1.0f) / (2.0f * Pi<float>())) * Li * pow(RdotV, alpha);
            local += lightContrib;
        }
    }

    // Indirect illumination
    Color indirect(0.0f);
    Color causticTerm2(0.0f);
    const PhotonMap* indirectPmap2 = nullptr;
    const PhotonMap* causticsPmap2 = nullptr;
    bool useIndirectMap2 = false;
    if (sInfoAd2 && USE_PHOTON_MAP_INDIRECT) {
        indirectPmap2 = sInfoAd2->GetPhotonMap();
        causticsPmap2 = sInfoAd2->GetCausticsMap();
        useIndirectMap2 = (indirectPmap2 && indirectPmap2->NumPhotons() > 0);
    }

    const bool hasDiffuse2 = (sInfo.Eval(Diffuse()).Max() > 1e-6f);

    // Always allow caustics map on diffuse surfaces so caustics remain visible through specular transport
    if (hasDiffuse2 && sInfoAd2 && causticsPmap2 && causticsPmap2->NumPhotons() > 0) {
        Color causticsIrrad = GetPhotonMapIrradiance(sInfo, causticsPmap2, sInfoAd2->GetCausticSearchRadius(), sInfoAd2->GetPhotonSearchCount());
        causticTerm2 = (sInfo.Eval(Diffuse()) / Pi<float>()) * causticsIrrad;
    }

    const bool canDiffuseBounce2 = hasDiffuse2 && sInfo.CanBounce();
    if (sInfoAd2 && useIndirectMap2 && !sInfoAd2->IsFirstDiffuseBounce()) {
        Color indirectIrrad = GetPhotonMapIrradiance(sInfo, indirectPmap2, sInfoAd2->GetPhotonSearchRadius(), sInfoAd2->GetPhotonSearchCount());
        indirect += (sInfo.Eval(Diffuse()) / Pi<float>()) * indirectIrrad;
    }
    else if (canDiffuseBounce2) {
        Vec3f indirDir;
        DirSampler::Info si;
        if (GenerateSample(sInfo, indirDir, si)) {
            float dist;
            Color Li = sInfo.TraceSecondaryRay(indirDir, dist, false);
            indirect = Li * si.mult / si.prob;
        }
    }
    local += (indirect + causticTerm2);

    const Color refl = sInfo.Eval(Reflection());
    const Color refrRaw = sInfo.Eval(Refraction());
    const float ior = IOR();

    const bool isDielectric = (refrRaw.Max() > 1e-6f);
    Color refrTint = isDielectric ? (refrRaw.Max() > 1e-6f ? refrRaw : sInfo.Eval(Diffuse())) : Color(0, 0, 0);

    Color result = local;

    if (!USE_PHOTON_MAP_DIRECT || USE_PHOTON_MAP_INDIRECT) {
        if ((isDielectric || refl.Max() > 1e-4f) && sInfo.CanBounce()) {
            const float kr = isDielectric ? FresnelExact(I, N, ior, sInfo.IsFront()) : 1.0f;

            // Reflection (adaptive samples: perfect/mild gloss = fewer)
            const float alphaVal = sInfo.Eval(Glossiness());
            const int glossySamples = 1;
            Color reflectionColor = SampleGlossy(N, alphaVal, true, V, sInfo, glossySamples, Color(0, 0, 0));

            // Refraction
            Color refractionColor(0.0f);
            if (isDielectric) {
                float n1 = 1.0f, n2 = ior;
                Vec3f NN = N;
                float cosi = -(NN % I);
                if (!sInfo.IsFront()) { std::swap(n1, n2); cosi = -(NN % I); }
                const float eta = n1 / n2;
                const float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
                if (k >= 0.0f) {
                    Vec3f T = eta * I + (eta * cosi - sqrtf(k)) * NN;
                    T.Normalize();

                    const int glossySamples = 1;

                    Vec3f U, Vb;
                    BuildONB(T, U, Vb);

                    refractionColor = SampleGlossy(T, sInfo.Eval(Glossiness()), false, V, sInfo, glossySamples, Absorption());
                }
            }

            if (isDielectric) {
                const Color oneMinusRefl = Color(1.0f) - refl;
                result = local * oneMinusRefl
                    + reflectionColor * kr
                    + refractionColor * ((1.0f - kr) * refrTint);
            }
            else {
                result = (local + reflectionColor * refl);
            }
        }
    }
    Color emission = sInfo.Eval(Emission());
    result += emission;

    return result;
}

// Minimal lambert placeholder for microfacet scenes
Color MtlMicrofacet::Shade(ShadeInfo const& sInfo) const {
    const Vec3f N = sInfo.N().GetNormalized();

    Color result(0.0f);
    if (sInfo.CurrentBounce() == 0) {
        for (int i = 0; i < sInfo.NumLights(); ++i) {
            Vec3f Ldir;
            const Light* L = sInfo.GetLight(i);
            const Color Li = L->Illuminate(sInfo, Ldir);
            if (L->IsAmbient()) { result += sInfo.Eval(baseColor) * Li; continue; }
            Ldir.Normalize();
            const float NdotL = max(0.0f, N % Ldir);
            if (NdotL > 0.0f) result += sInfo.Eval(baseColor) * Li * NdotL;
        }
    }

    // Indirect illumination
    Color indirect(0.0f);
    if (sInfo.Eval(baseColor).Max() > 1e-6f && sInfo.CanBounce()) {
        Vec3f indirDir;
        DirSampler::Info si;
        if (GenerateSample(sInfo, indirDir, si)) {
            float dist;
            Color Li = sInfo.TraceSecondaryRay(indirDir, dist, false);
            indirect = Li * si.mult;
        }
    }
    result += indirect;

    result += sInfo.Eval(emission);

    return result;
}

// BRDF Sampling for Photon Mapping
bool MtlBlinn::GenerateSample(SamplerInfo const& sInfo, Vec3f& dir, DirSampler::Info& si) const {
    Vec3f N = sInfo.N().GetNormalized();
    if (!sInfo.IsFront()) N = -N;

    Vec3f V = sInfo.V().GetNormalized();
    float u1 = sInfo.RandomFloat();
    float u2 = sInfo.RandomFloat();

    Color diff = sInfo.Eval(Diffuse());
    Color refl = sInfo.Eval(Reflection());
    Color refr = sInfo.Eval(Refraction());
    float diffWeight = diff.Sum();
    float reflWeight = refl.Sum();
    float refrWeight = refr.Sum();
    float total = diffWeight + reflWeight + refrWeight;

    if (total < 1e-6f) return false;

    float pick = sInfo.RandomFloat() * total;

    if (pick < diffWeight) {
        // Diffuse lobe - cosine-weighted hemisphere sampling
        Vec3f U, Vb;
        BuildONB(N, U, Vb);
        float cosTheta = sqrtf(u1);
        float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
        float phi = 2.0f * float(M_PI) * u2;
        dir = U * (sinTheta * cosf(phi)) + Vb * (sinTheta * sinf(phi)) + N * cosTheta;

        // For cosine-weighted sampling:
           // BRDF = ρd/π, pdf = cosθ/π, cosθ from rendering equation
           // weight = BRDF * cosθ / pdf = (ρd/π) * cosθ / (cosθ/π) = ρd
        si.mult = diff;
        si.prob = diffWeight / total;  // Just the lobe selection probability
        si.lobe = DirSampler::Lobe::DIFFUSE;
    }
    else if (pick < diffWeight + reflWeight) {
        // Specular reflection lobe - Phong/Blinn cosine lobe sampling
        float alpha = sInfo.Eval(Glossiness());
        Vec3f R = 2.0f * (N % V) * N - V;
        R.Normalize();

        Vec3f U, Vb;
        BuildONB(R, U, Vb);
        float cosTheta = powf(u1, 1.0f / (alpha + 1.0f));
        float sinTheta = sqrtf(max(0.0f, 1.0f - cosTheta * cosTheta));
        float phi = 2.0f * float(M_PI) * u2;
        Vec3f H = U * (sinTheta * cosf(phi)) + Vb * (sinTheta * sinf(phi)) + R * cosTheta;
        H.Normalize();

        dir = 2.0f * (V % H) * H - V;
        dir.Normalize();

        // For perfect specular, just return the material reflectance
        // (The actual BRDF would be a delta function, but for sampling we use the albedo)
        si.mult = refl;
        si.prob = reflWeight / total;
        si.lobe = DirSampler::Lobe::SPECULAR;
    }
    else {
        // Refraction lobe
        Vec3f I = -V;
        float n1 = 1.0f, n2 = IOR();
        if (!sInfo.IsFront()) std::swap(n1, n2);
        float eta = n1 / n2;
        float cosi = -(N % I);
        float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
        if (k < 0.0f) return false;  // Total internal reflection

        dir = eta * I + (eta * cosi - sqrtf(k)) * N;
        dir.Normalize();

        Vec3f T = dir;
        float alpha = sInfo.Eval(Glossiness());
        if (alpha > 0.0f) {
            Vec3f U, Vb;
            BuildONB(T, U, Vb);
            float cosTheta = powf(u1, 1.0f / (alpha + 1.0f));
            float sinTheta = sqrtf(max(0.0f, 1.0f - cosTheta * cosTheta));
            float phi = 2.0f * float(M_PI) * u2;
            dir = U * (sinTheta * cosf(phi)) + Vb * (sinTheta * sinf(phi)) + T * cosTheta;
            dir.Normalize();
        }

        // For perfect specular transmission, return the transmittance
        si.mult = refr;
        si.prob = refrWeight / total;
        si.lobe = DirSampler::Lobe::TRANSMISSION;
    }

    return true;
}

bool MtlPhong::GenerateSample(SamplerInfo const& sInfo, Vec3f& dir, DirSampler::Info& si) const {
    Vec3f N = sInfo.N().GetNormalized();
    if (!sInfo.IsFront()) N = -N;

    Vec3f V = sInfo.V().GetNormalized();
    float u1 = sInfo.RandomFloat();
    float u2 = sInfo.RandomFloat();

    Color diff = sInfo.Eval(Diffuse());
    Color refl = sInfo.Eval(Reflection());
    Color refr = sInfo.Eval(Refraction());

    float diffWeight = diff.Sum();
    float reflWeight = refl.Sum();
    float refrWeight = refr.Sum();
    float total = diffWeight + reflWeight + refrWeight;

    if (total < 1e-6f) return false;

    float pick = sInfo.RandomFloat() * total;

    if (pick < diffWeight) {
        // Diffuse lobe - cosine-weighted hemisphere sampling
        Vec3f U, Vb;
        BuildONB(N, U, Vb);
        float cosTheta = sqrtf(u1);
        float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
        float phi = 2.0f * float(M_PI) * u2;
        dir = U * (sinTheta * cosf(phi)) + Vb * (sinTheta * sinf(phi)) + N * cosTheta;

        // Cosine-weighted: weight = ρd (BRDF * cosθ / pdf cancels to just ρd)
        si.mult = diff;
        si.prob = diffWeight / total;
        si.lobe = DirSampler::Lobe::DIFFUSE;
    }
    else if (pick < diffWeight + reflWeight) {
        // Specular reflection lobe  
        float alpha = sInfo.Eval(Glossiness());
        Vec3f R = 2.0f * (N % V) * N - V;
        R.Normalize();

        Vec3f U, Vb;
        BuildONB(R, U, Vb);
        float cosTheta = powf(u1, 1.0f / (alpha + 1.0f));
        float sinTheta = sqrtf(max(0.0f, 1.0f - cosTheta * cosTheta));
        float phi = 2.0f * float(M_PI) * u2;
        dir = U * (sinTheta * cosf(phi)) + Vb * (sinTheta * sinf(phi)) + R * cosTheta;
        dir.Normalize();

        si.mult = refl;
        si.prob = reflWeight / total;
        si.lobe = DirSampler::Lobe::SPECULAR;
    }
    else {
        // Refraction lobe
        Vec3f I = -V;
        float n1 = 1.0f, n2 = IOR();
        if (!sInfo.IsFront()) std::swap(n1, n2);
        float eta = n1 / n2;
        float cosi = -(N % I);
        float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
        if (k < 0.0f) return false;

        dir = eta * I + (eta * cosi - sqrtf(k)) * N;
        dir.Normalize();

        Vec3f T = dir;
        float alpha = sInfo.Eval(Glossiness());
        if (alpha > 0.0f) {
            Vec3f U, Vb;
            BuildONB(T, U, Vb);
            float cosTheta = powf(u1, 1.0f / (alpha + 1.0f));
            float sinTheta = sqrtf(max(0.0f, 1.0f - cosTheta * cosTheta));
            float phi = 2.0f * float(M_PI) * u2;
            dir = U * (sinTheta * cosf(phi)) + Vb * (sinTheta * sinf(phi)) + T * cosTheta;
            dir.Normalize();
        }

        si.mult = refr;
        si.prob = refrWeight / total;
        si.lobe = DirSampler::Lobe::TRANSMISSION;
    }

    return true;
}

bool MtlMicrofacet::GenerateSample(SamplerInfo const& sInfo, Vec3f& dir, DirSampler::Info& si) const {
    Vec3f N = sInfo.N().GetNormalized();

    // Simple cosine-weighted diffuse sampling
    Vec3f U, Vb;
    BuildONB(N, U, Vb);

    float u1 = sInfo.RandomFloat();
    float u2 = sInfo.RandomFloat();

    float cosTheta = sqrtf(u1);
    float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
    float phi = 2.0f * float(M_PI) * u2;

    dir = U * (sinTheta * cosf(phi)) + Vb * (sinTheta * sinf(phi)) + N * cosTheta;

    si.mult = sInfo.Eval(baseColor);
    si.prob = cosTheta / Pi<float>();
    si.lobe = DirSampler::Lobe::DIFFUSE;

    return true;
}

#endif // SHADING_USE_LEGACY
