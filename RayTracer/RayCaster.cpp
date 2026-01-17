#include "RayCaster.h"
#include "scene.h"
#include "objects.h"
#include "CameraAdapter.h"
#include "IntersectNodeAdapter.h"
#include "RenderImageAdapter.h"
#include "ShadingConfig.h"
#if SHADING_USE_LEGACY
#include "ShadeInfoAdapter.h"
#endif
#include "BumpMapping.h"
#include "materials.h"
#include "rng.h"
#include "PhotonMapBuilder.h"
#include "Denoiser.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstdint>
#include <chrono>
#include <thread>

// Define M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Global filter selection
FilterType g_filterType = FilterType::Tent;
float      g_filterRadius = 1.0f;

static inline Color24 ToColor24(const Color& cLin, bool srgb) {
    Color c;
    if (srgb) {
        c = cLin.Linear2sRGB();
    }
    else {
        c = cLin;
    }
    c.ClampMin(0.0f);
    c.ClampMax(1.0f);
    return Color24(
        static_cast<uint8_t>(c.r * 255.0f),
        static_cast<uint8_t>(c.g * 255.0f),
        static_cast<uint8_t>(c.b * 255.0f)
    );
}

// Antialiasing parameters
static constexpr int kMinSamples = 128;      // minimum samples per pixel
static constexpr int kInitSamples = 128;     // initial samples used before adaptive decision
static constexpr int kMaxSamples = 1024;     // maximum samples per pixel
static constexpr float kConfidenceLevel = 0.95f; // confidence level for adaptive sampling (e.g., 0.95 for 95%)
static constexpr float kMaxHalfWidth = 0.005f;    // maximum half-width of confidence interval
static constexpr float kNeighborColorDiff = 0.05f; // threshold for neighbor color deviation

// Default filter radius fallback (used if g_filterRadius <= 0)
static constexpr float kDefaultTentRadius = 1.0f;

// Progressive preview sampling budget per pass
static constexpr int kProgressiveSamplesPerPass = 1;

// splitmix64 for deterministic per-pixel hashing
static inline uint64_t splitmix64(uint64_t& x) {
    uint64_t z = (x += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

// Generate two uniform [0,1) shifts for a pixel (Cranley-Patterson rotation)
static inline void PixelShifts(int x, int y, float& shiftX, float& shiftY) {
    uint64_t seed = (uint64_t(uint32_t(x)) << 32) | uint64_t(uint32_t(y));
    uint64_t s = seed;
    uint64_t h1 = splitmix64(s);
    uint64_t h2 = splitmix64(s);
    // convert lower 32 bits to fraction
    shiftX = float((h1 & 0xFFFFFFFFULL) / double(0x100000000ULL));
    shiftY = float((h2 & 0xFFFFFFFFULL) / double(0x100000000ULL));
}

// Default support per filter
static inline float FilterDefaultSupport(FilterType t) {
    switch (t) {
    case FilterType::Tent: return 1.0f;
    case FilterType::Gaussian: return 1.5f;
    case FilterType::Mitchell: return 2.0f;
    case FilterType::Lanczos: return 2.0f; // common choice
    }
    return 1.0f;
}

static inline float Clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

static inline Vec3f SafeNormalize(Vec3f v) {
    float len2 = v.LengthSquared();
    if (len2 <= 0.0f) return Vec3f(0, 0, 0);
    return v / std::sqrt(len2);
}

static inline float PowerHeuristic(float pdfA, float pdfB) {
    float a = pdfA * pdfA;
    float b = pdfB * pdfB;
    return a / (a + b);
}

static inline Vec3f OffsetPoint(Vec3f const& p, Vec3f const& n, Vec3f const& dir) {
    Vec3f nn = SafeNormalize(n);
    float sign = (dir % nn) >= 0.0f ? 1.0f : -1.0f;
    return p + nn * (1e-4f * sign);
}

// Clamp very small PDFs to avoid huge weights that cause fireflies
static constexpr float kMinPdf = 1e-4f;

// Optional radiance clamp to tame fireflies from rare high-energy paths (set to 0 to disable)
static constexpr float kRadianceClamp = 10.0f;

// Simple SamplerInfo wrapper that lets us set the bounce counter.
class PathSamplerInfo : public SamplerInfo {
public:
    PathSamplerInfo(RNG& r) : SamplerInfo(r) {}
    void SetBounce(int b) { bounce = b; }
};

struct PathSampleResult {
    Color radiance;
    float primaryDist;
    Color albedo;
    Vec3f normal;
    bool hit = false;
};

static Color GetDiffuseAlbedo(const Material* mtl, SamplerInfo const& sInfo) {
    if (auto* p = dynamic_cast<MtlBasePhongBlinn const*>(mtl)) return sInfo.Eval(p->Diffuse());
    return Color(0.5f); // fallback for other materials
}

#if !SHADING_USE_LEGACY

// Estimate caustic contribution from photon map (if available)
static Color EstimateCaustics(const PhotonMapBuilder* mapper, PhotonMap const* map, HitInfo const& h) {
    if (!mapper || !map || map->NumPhotons() == 0) return Color(0, 0, 0);
    Color irrad;
    Vec3f avgDir;
    float radius = mapper->GetCausticSearchRadius();
    int maxPhotons = mapper->GetPhotonSearchCount();
    if (maxPhotons <= 30) map->EstimateIrradiance<30, PHOTONMAP_FILTER_LINEAR>(irrad, avgDir, radius, h.p, h.N);
    else if (maxPhotons <= 50) map->EstimateIrradiance<50, PHOTONMAP_FILTER_LINEAR>(irrad, avgDir, radius, h.p, h.N);
    else map->EstimateIrradiance<100, PHOTONMAP_FILTER_LINEAR>(irrad, avgDir, radius, h.p, h.N);
    return irrad;
}

// Tunable caustic boost to bring photon map energy up to visible levels in the path tracer.
static constexpr float kCausticBoost = 0.1f;

static Color GetEmission(const Material* mtl, SamplerInfo const& sInfo) {
    if (auto* p = dynamic_cast<MtlBasePhongBlinn const*>(mtl)) return sInfo.Eval(p->Emission());
    return Color(0, 0, 0); // microfacet emission unavailable without framework changes
}

// Helper to check if a material is refractive (glass-like)
static bool IsRefractiveMaterial(const Material* mtl, int mtlID) {
    if (!mtl) return false;

    // Check if it's a Phong/Blinn material with refraction
    if (auto* phong = dynamic_cast<const MtlBasePhongBlinn*>(mtl)) {
        Color refraction = phong->Refraction().GetValue();
        // Material is refractive if it has non-zero refraction color
        return (refraction.r > 0.0f || refraction.g > 0.0f || refraction.b > 0.0f);
    }

    return false;
}

// Fast boolean visibility check (no transmittance)
static bool VisibleSimple(const Scene& scene, Vec3f const& origin, Vec3f const& normal, Vec3f const& dir, float maxDist) {
    Ray shadowRay(OffsetPoint(origin, normal, dir), SafeNormalize(dir));
    ExtendedHitInfo h; h.Init();
    h.z = maxDist;
    
    if (!IntersectSceneWithLights(&scene, shadowRay, h, HIT_FRONT_AND_BACK)) return true; // Nothing blocking
    if (h.light) return true; // Hit the light itself
    
    float dist = (h.p - shadowRay.p).Length();
    if (dist >= maxDist - 1e-4f) return true; // Beyond light
    
    return false; // Blocked by geometry
}

// Transmittance-based visibility (for caustics through glass)
static float Visible(const Scene& scene, Vec3f const& origin, Vec3f const& normal, Vec3f const& dir, float maxDist, bool useTransmittance = true) {
    // Fast path: simple boolean check when transmittance not needed
    if (!useTransmittance) {
        return VisibleSimple(scene, origin, normal, dir, maxDist) ? 1.0f : 0.0f;
    }
    
    Ray shadowRay(OffsetPoint(origin, normal, dir), SafeNormalize(dir));

    const int kMaxHops = 8;
    float visibility = 1.0f;

    for (int hop = 0; hop < kMaxHops; ++hop) {
        ExtendedHitInfo h; h.Init();
        // Clamp intersection distance to the remaining segment to the light
        h.z = maxDist;

        if (!IntersectSceneWithLights(&scene, shadowRay, h, HIT_FRONT_AND_BACK)) return visibility;

        // If we hit the light source itself, it's visible with accumulated transmittance
        if (h.light) return visibility;

        float dist = (h.p - shadowRay.p).Length();
        // If the hit is beyond the target light point, treat as clear
        if (dist >= maxDist - 1e-4f) return visibility;

        // Fast early-out for opaque materials - check transmissiveness first
        bool isTransmissive = false;
        float transFactor = 0.0f;
        float ior = 1.5f;
        
        if (h.node) {
            if (const Material* m = h.node->GetMaterial()) {
                // Quick check: is this material transmissive at all?
                if (auto* pb = dynamic_cast<const MtlBasePhongBlinn*>(m)) {
                    Color kt = pb->Refraction().GetValue();
                    transFactor = kt.Max();
                    if (transFactor <= 0.0f) return 0.0f; // Opaque - early exit
                    ior = pb->IOR(h.mtlID);
                    isTransmissive = true;
                } else if (auto* mf = dynamic_cast<const MtlMicrofacet*>(m)) {
                    Color t = mf->Transmittance().GetValue();
                    transFactor = t.Max();
                    if (transFactor <= 0.0f) return 0.0f; // Opaque - early exit
                    ior = mf->IOR(h.mtlID);
                    isTransmissive = true;
                } else {
                    // Unknown material type or no transmittance - treat as opaque
                    return 0.0f;
                }
            } else {
                // No material - treat as opaque
                return 0.0f;
            }
        } else {
            // No node - treat as opaque
            return 0.0f;
        }
        
        // At this point, we know the surface is transmissive
        auto fresnelSchlick = [](float cosTheta, float etaI, float etaT) {
            cosTheta = std::clamp(cosTheta, 0.0f, 1.0f);
            float r0 = (etaI - etaT) / (etaI + etaT);
            r0 = r0 * r0;
            return r0 + (1.0f - r0) * std::pow(1.0f - cosTheta, 5.0f);
        };

        // Apply a Fresnel-based transmission factor so glass still attenuates direct light
        float cosTheta = fabsf(h.GN % (-shadowRay.dir));
        float etaI = h.front ? 1.0f : ior;
        float etaT = h.front ? ior : 1.0f;
        float F = fresnelSchlick(cosTheta, etaI, etaT);
        // Clamp to [0,1] to avoid any energy gain in shadows
        float transmit = std::max(0.0f, std::min(1.0f, (1.0f - F) * transFactor));
        visibility *= std::max(0.0f, std::min(1.0f, transmit));
        if (visibility <= 1e-4f) return 0.0f;

        // Continue the shadow ray past the transmissive surface
        shadowRay.p = OffsetPoint(h.p, h.GN, shadowRay.dir);
        // Keep the same direction; maxDist reduces by traveled distance
        maxDist -= dist;
        if (maxDist <= 1e-4f) return visibility;
    }

    // Safety: if we exceed hop limit, assume blocked
    return 0.0f;
}

// Apply per-channel Beer-Lambert absorption to a color given absorption coefficients and distance.
static inline Color ApplyAbsorption(const Color& c, const Color& absorption, float dist) {
    if (dist <= 0.0f) return c;
    // If absorption is effectively zero, return original color to preserve current appearance
    if (absorption.r == 0.0f && absorption.g == 0.0f && absorption.b == 0.0f) return c;
    
    // Safety: absorption coefficients should be positive (negative would amplify light)
    float absR = std::max(0.0f, absorption.r);
    float absG = std::max(0.0f, absorption.g);
    float absB = std::max(0.0f, absorption.b);
    
    const float ar = expf(-absR * dist);
    const float ag = expf(-absG * dist);
    const float ab = expf(-absB * dist);
    
    // Result should always be <= input (attenuation only, never amplification)
    return Color(
        std::min(c.r, c.r * ar),
        std::min(c.g, c.g * ag),
        std::min(c.b, c.b * ab)
    );
}

static inline bool DirToScreenUV(const Camera& cam, Vec3f const& dir, float& u, float& v) {
    // Map a direction to the camera's screen UV (0..1) using the same projection as ray generation.
    Vec3f f = cam.dir;
    Vec3f r = (f.Cross(cam.up)).GetNormalized();
    Vec3f uvec = (r.Cross(f)).GetNormalized();

    float dz = dir % f;
    if (fabs(dz) < 1e-6f) dz = (dz >= 0.0f) ? 1e-6f : -1e-6f; // avoid divide-by-zero

    float sx = (dir % r) / dz;
    float sy = (dir % uvec) / dz;

    float fovRad = cam.fov * float(M_PI) / 180.0f;
    float halfY = tanf(0.5f * fovRad);
    float aspect = (cam.imgHeight > 0) ? float(cam.imgWidth) / float(cam.imgHeight) : 1.0f;
    float halfX = halfY * aspect;

    float ndcX = sx / halfX;         // -1..1
    float ndcY = sy / halfY;         // -1..1

    u = (ndcX + 1.0f) * 0.5f;        // 0..1 left->right
    v = (1.0f - ndcY) * 0.5f;        // 0..1 bottom->top (match original background sampling)

    // wrap horizontally; clamp vertically to avoid crazy values for grazing directions
    u = u - floorf(u);
    v = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    return true;
}

// Returns true if the textured color has either a texture map or a non-black base value.
static inline bool HasMapOrValue(const TexturedColor& tc) {
    const TextureMap* tex = tc.GetTexture();
    const Color base = tc.GetValue();
    return tex != nullptr || base.r != 0.0f || base.g != 0.0f || base.b != 0.0f;
}

static PathSampleResult TracePath(const Scene& scene, const Camera& camera, Ray ray, RNG& rng, int px, int py, int sampleIdx, const PhotonMapBuilder* mapper) {
    Color L(0, 0, 0);
    Color beta(1, 1, 1);
    float primaryDist = BIGFLOAT;
    Color firstAlbedo(0, 0, 0);
    Vec3f firstNormal(0, 0, 0);
    bool haveFirstHit = false;
    bool firstHit = true;
    bool lastDelta = true; // Only true for delta events (e.g., perfect refraction); glossy reflection is not delta
    float lastPdf = 1.0f;
    PathSamplerInfo sInfoA(rng);
    PathSamplerInfo sInfoB(rng);
    PathSamplerInfo* prevSInfoPtr = nullptr;
    bool hasPrevSInfo = false;
    
    // Track absorption: when inside a medium, store its absorption coefficient
    bool insideMedium = false;
    Color mediumAbsorption(0.0f);

    const int maxBounces = 64;
    for (int bounce = 0; bounce < maxBounces; ++bounce) {
        ExtendedHitInfo h; h.Init();
        bool hitGeom = IntersectSceneWithLights(&scene, ray, h, HIT_FRONT_AND_BACK);
        if (!hitGeom) {
            float uEnv = 0.0f, vEnv = 0.0f;
            DirToScreenUV(camera, ray.dir.GetNormalized(), uEnv, vEnv);
            // Visible backplate: use <background> if provided, otherwise fall back to the environment map.
            // Background can optionally emit (flat) via backgroundEmission; environment stays spherical.
            const bool hasBackground = HasMapOrValue(scene.background);
            const bool hasEnvironment = HasMapOrValue(scene.environment);
            const Color backplate = hasBackground ? scene.background.Eval(Vec3f(uEnv, vEnv, 0.0f)) : Color(0, 0, 0);
            const Color envLight = hasEnvironment ? scene.environment.EvalEnvironment(ray.dir.GetNormalized()) : Color(0, 0, 0);
            const Color bgEmit = (hasBackground && scene.backgroundEmission > 0.0f) ? backplate * scene.backgroundEmission : Color(0, 0, 0);

            if (bounce == 0) {
                // Only the camera ray sees the backplate; lighting comes from the environment
                const Color visible = hasBackground ? backplate : envLight;
                L += beta * (visible + bgEmit);
            } else {
                // Secondary rays gather energy from the environment map and optional flat backplate emission
                L += beta * (envLight + bgEmit);
            }
            break;
        }

        float hitDist = (h.p - ray.p).Length();
        
        // Apply Beer-Lambert absorption if we're inside a medium
        if (insideMedium && hitDist > 0.0f) {
            beta = ApplyAbsorption(beta, mediumAbsorption, hitDist);
        }

        PathSamplerInfo* currSInfoPtr = hasPrevSInfo ? (prevSInfoPtr == &sInfoA ? &sInfoB : &sInfoA) : &sInfoA;
        PathSamplerInfo& sInfo = *currSInfoPtr;
        sInfo.SetPixel(px, py);
        sInfo.SetPixelSample(sampleIdx);
        sInfo.SetBounce(bounce);

        const Material* mtl = h.node ? h.node->GetMaterial() : nullptr;
        ApplyBumpMapping(mtl, h);
        sInfo.SetHit(ray, h);

        if (firstHit) {
            primaryDist = hitDist;
            firstHit = false;
            firstAlbedo = GetDiffuseAlbedo(mtl, sInfo);
            firstNormal = SafeNormalize(h.GN);
            haveFirstHit = true;
        }

        // If we hit a renderable light directly
        if (h.light && h.hitLight) {
            const Light* light = h.hitLight;
            Color emitted = light->Radiance(sInfo);
            // Apply MIS weight: balance between BSDF sampling (this path) and light sampling (NEE)
            // Use the previous surface's sampling context for the light PDF; using the light's own hit
            // point would incorrectly drive the PDF to zero and over-weight this contribution
            float lightPdfSA = 0.0f;
            if (hasPrevSInfo) {
                DirSampler::Info li;
                light->GetSampleInfo(*prevSInfoPtr, ray.dir, li);
                lightPdfSA = li.prob;
                if (light->IsRenderable()) {
                    float cosLight = std::max(0.0f, -h.N % SafeNormalize(ray.dir));
                    float dist2 = li.dist * li.dist;
                    if (cosLight > 1e-6f && dist2 > 0.0f) {
                        lightPdfSA = li.prob * dist2 / cosLight;
                    }
                    else {
                        lightPdfSA = 0.0f;
                    }
                }
                else {
                    // Delta/point lights: treat as pure delta so MIS weight stays 1 on hit
                    lightPdfSA = 0.0f;
                }
            }
            float w = (lastDelta || lightPdfSA <= 0.0f) ? 1.0f : PowerHeuristic(lastPdf, lightPdfSA);
            L += beta * emitted * w;
            break;
        }

        if (!mtl) break;

        // Add emission if the material has any (but not for lights, as they're handled above)
        // Note: This handles emissive materials that are not light objects
        Color emission = GetEmission(mtl, sInfo);
        if (emission.Max() > 0.0f) L += beta * emission;

        // Next event estimation with MIS
        Color direct(0, 0, 0);
        float maxVisibility = 0.0f; // Track visibility for caustic modulation
        
        // Use transmittance only on early bounces where caustics matter most
        // This gives good visual quality while keeping performance high
        const bool useTransmittance = (bounce <= 8);
        
        for (Light const* light : scene.lights) {
            Vec3f ldir;
            DirSampler::Info li;
            if (!light->GenerateSample(sInfo, ldir, li)) continue;
            if (li.prob <= 0.0f || li.mult.Max() <= 0.0f) continue;

            float shadowMax = (li.dist > 0.0f && li.dist < BIGFLOAT * 0.5f) ? li.dist : BIGFLOAT;
            
            // Get transmittance or binary visibility based on bounce depth
            float transmittance = Visible(scene, h.p, sInfo.GN(), ldir, shadowMax, useTransmittance);
            
            // Track the maximum visibility across all lights for caustic modulation
            maxVisibility = std::max(maxVisibility, transmittance);
            
            if (transmittance <= 0.0f) continue; // Fully occluded

            DirSampler::Info bsdf;
            mtl->GetSampleInfo(sInfo, SafeNormalize(ldir), bsdf);
            if (bsdf.prob <= 0.0f || bsdf.mult.Max() <= 0.0f) continue;

            float w = 1.0f;
            if (light->IsRenderable()) {
                // For area lights: convert PDF from area measure to solid angle and use that consistently
                float dist = li.dist;
                float cosLight = 1.0f;

                Ray lightRay(h.p, SafeNormalize(ldir));
                HitInfo lightHit; lightHit.Init();
                if (light->IntersectRay(lightRay, lightHit, HIT_FRONT)) {
                    cosLight = std::max(0.0f, -lightHit.N % SafeNormalize(ldir));
                }

                float pdfSolidAngle = (cosLight > 1e-6f && dist > 1e-6f) ? li.prob * dist * dist / cosLight : 0.0f;
                if (pdfSolidAngle <= 0.0f) continue;

                w = PowerHeuristic(pdfSolidAngle, bsdf.prob);

                // NEE estimator: Li * BSDF * cosTheta * G / pdf_area, where G = 1/dist^2
                // li.mult is radiance, bsdf.mult includes cosTheta
                float G = 1.0f / std::max(1e-6f, dist * dist);
                direct += li.mult * bsdf.mult * G * (w / std::max(1e-6f, li.prob)) * transmittance;
            }
            else {
                // Delta/point lights: li.mult already includes distance attenuation; avoid duplicating G
                w = PowerHeuristic(li.prob, bsdf.prob);
                direct += li.mult * bsdf.mult * (w / std::max(kMinPdf, li.prob)) * transmittance; // Multiply by transmittance
            }
        }
        if (direct.Max() > 0.0f) L += beta * direct;
        
        // Optional caustics map (only on diffuse-like surfaces)
        // Modulate caustics by visibility to prevent them from overpowering shadows
        if (mapper) {
            PhotonMap const* causticMap = mapper->GetCausticsMap();
            if (causticMap && causticMap->NumPhotons() > 0 && mtl->IsPhotonSurface(h.mtlID)) {
                Color irrad = EstimateCaustics(mapper, causticMap, h);
                if (irrad.Max() > 0.0f) {
                    Color kd = GetDiffuseAlbedo(mtl, sInfo);
                    // Boost slightly so the photon caustics stand out in the final render
                    irrad *= kCausticBoost;
                    // Modulate by visibility: caustics are stronger where direct light is present
                    // Use a soft transition: mix between 0.1 (in shadow) and 1.0 (in light)
                    float causticScale = 0.1f + 0.9f * maxVisibility;
                    L += beta * (kd * (1.0f / float(M_PI)) * irrad * causticScale);
                }
            }
        }

        // Sample BSDF for the next bounce
        Vec3f nextDir;
        DirSampler::Info bsdfSample;
        if (!mtl->GenerateSample(sInfo, nextDir, bsdfSample)) {
            break;
        }
        if (bsdfSample.prob <= 0.0f || bsdfSample.mult.Max() <= 0.0f) {
            break;
        }

        // Clamp PDF in throughput to keep extremely rare events from exploding into fireflies.
        float pdfThroughput = std::max(kMinPdf, bsdfSample.prob);
        beta = beta * (bsdfSample.mult / pdfThroughput);
        lastPdf = bsdfSample.prob;
        // Only true delta events (e.g., perfect refraction) should disable MIS on the next light hit
        // Glossy reflection is not delta
        lastDelta = (bsdfSample.dist < 0.0f);
        prevSInfoPtr = currSInfoPtr;
        hasPrevSInfo = true;
        
        // Update medium tracking for absorption (Beer-Lambert law)
        if (bsdfSample.lobe & DirSampler::Lobe::TRANSMISSION) {
            if (sInfo.IsFront()) {
                // Entering a medium - store its absorption coefficient
                insideMedium = true;
                mediumAbsorption = mtl->Absorption(h.mtlID);
            } else {
                // Exiting a medium - back to air/vacuum
                insideMedium = false;
                mediumAbsorption = Color(0.0f);
            }
        }

        // Russian roulette after a few bounces
        if (bounce >= 5) {
            float q = Clamp01(beta.Max());
            q = std::max(q, 0.05f);
            if (rng.RandomFloat() > q) break;
            beta *= 1.0f / q;
        }

        ray.p = OffsetPoint(h.p, sInfo.GN(), nextDir);
        ray.dir = SafeNormalize(nextDir);
    }

    if (kRadianceClamp > 0.0f) {
        L.ClampMax(kRadianceClamp);
    }

    return { L, primaryDist, firstAlbedo, firstNormal, haveFirstHit };
}

#endif // !SHADING_USE_LEGACY

#if SHADING_USE_LEGACY
static PathSampleResult TraceLegacy(const Scene& scene, const Camera& camera, Ray ray, RNG& rng, int px, int py, int sampleIdx, const PhotonMapBuilder* mapper) {
    PathSampleResult res{ Color(0, 0, 0), BIGFLOAT, Color(0, 0, 0), Vec3f(0, 0, 0), false };

    ExtendedHitInfo h; h.Init();
    if (!IntersectSceneWithLights(&scene, ray, h, HIT_FRONT_AND_BACK)) {
        float uEnv = 0.0f, vEnv = 0.0f;
        DirToScreenUV(camera, ray.dir.GetNormalized(), uEnv, vEnv);
        const bool hasBackground = HasMapOrValue(scene.background);
        const bool hasEnvironment = HasMapOrValue(scene.environment);
        const Color backplate = hasBackground ? scene.background.Eval(Vec3f(uEnv, vEnv, 0.0f)) : Color(0, 0, 0);
        const Color envLight = hasEnvironment ? scene.environment.EvalEnvironment(ray.dir.GetNormalized()) : Color(0, 0, 0);
        const Color bgEmit = (hasBackground && scene.backgroundEmission > 0.0f) ? backplate * scene.backgroundEmission : Color(0, 0, 0);
        const Color visible = hasBackground ? backplate : envLight;
        res.radiance = visible + bgEmit;
        return res;
    }

    res.primaryDist = (h.p - ray.p).Length();

    ShadeInfoAdapter sInfo(scene.lights, &scene, rng);
    sInfo.SetPixel(px, py);
    sInfo.SetPixelSample(sampleIdx);
    const Material* mtl = h.node ? h.node->GetMaterial() : nullptr;
    ApplyBumpMapping(mtl, h);
    sInfo.SetHit(ray, h);

    if (mapper) {
        sInfo.SetPhotonMap(mapper->GetPhotonMap());
        sInfo.SetCausticsMap(mapper->GetCausticsMap());
        sInfo.SetPhotonSearchRadius(mapper->GetPhotonSearchRadius());
        sInfo.SetCausticSearchRadius(mapper->GetCausticSearchRadius());
        sInfo.SetPhotonSearchCount(mapper->GetPhotonSearchCount());
    }

    if (h.light) {
        const Light* light = h.hitLight;
        if (!light && h.node) {
            if (const Object* obj = h.node->GetNodeObj()) {
                light = dynamic_cast<const Light*>(obj);
            }
        }
        if (light) {
            res.radiance = light->Radiance(sInfo);
            return res;
        }
    }

    res.radiance = mtl ? mtl->Shade(sInfo) : Color(0, 0, 0);
    res.albedo = mtl ? GetDiffuseAlbedo(mtl, sInfo) : Color(0, 0, 0);
    res.normal = SafeNormalize(h.GN);
    res.hit = true;
    return res;
}
#endif

static inline PathSampleResult TraceRadiance(const Scene& scene, const Camera& cam, Ray ray, RNG& rng, int px, int py, int sampleIdx, const PhotonMapBuilder* mapper) {
#if SHADING_USE_LEGACY
    return TraceLegacy(scene, cam, ray, rng, px, py, sampleIdx, mapper);
#else
    return TracePath(scene, cam, ray, rng, px, py, sampleIdx, mapper);
#endif
}

// Tent (triangle) filter
static inline float TentFilter(float x, float radius) {
    x = fabs(x);
    if (x >= radius) return 0.0f;
    return 1.0f - x / radius;
}

// Gaussian 1D (truncated at radius)
static inline float GaussianFilter(float x, float radius) {
    float sigma = radius * 0.5f;
    if (sigma <= 0.0f) sigma = 0.5f;
    float t = x / sigma;
    return expf(-0.5f * t * t);
}

// Mitchell-Netravali 1D (support is 2)
static inline float Mitchell1D(float x) {
    const float B = 1.0f / 3.0f;
    const float C = 1.0f / 3.0f;
    x = fabs(x);
    if (x < 1.0f) {
        return ((12.0f - 9.0f * B - 6.0f * C) * x * x * x + (-18.0f + 12.0f * B + 6.0f * C) * x * x + (6.0f - 2.0f * B)) / 6.0f;
    }
    else if (x < 2.0f) {
        return ((-B - 6.0f * C) * x * x * x + (6.0f * B + 30.0f * C) * x * x + (-12.0f * B - 48.0f * C) * x + (8.0f * B + 24.0f * C)) / 6.0f;
    }
    return 0.0f;
}

// Lanczos 1D (a is radius, support a)
static inline float Lanczos1D(float x, float a) {
    x = fabs(x);
    if (x >= a) return 0.0f;
    if (x < 1e-6f) return 1.0f;
    float pix = float(M_PI) * x;
    return (a * sinf(pix) * sinf(pix / a)) / (pix * pix);
}

static inline float Filter1D(float x) {
    // effective radius for kernels that need it
    float radius = (g_filterRadius > 0.0f) ? g_filterRadius : FilterDefaultSupport(g_filterType);
    switch (g_filterType) {
    case FilterType::Tent:     return TentFilter(x, radius);
    case FilterType::Gaussian: return GaussianFilter(x, radius);
    case FilterType::Mitchell: return Mitchell1D(x); // Mitchell expects x in pixels, support=2
    case FilterType::Lanczos:  return Lanczos1D(x, radius);
    }
    return TentFilter(x, radius);
}

RayCaster::RayCaster(Scene* scene, Camera* camera)
    : scene(scene), camera(camera)
{
}

void RayCaster::RenderBucket(RenderImage& image, const Bucket& b,
    std::vector<Color>& accumColor,
    std::vector<float>& accumWeight,
    std::vector<int>& originSampleCount,
    std::vector<float>& accumZ,
    std::vector<uint8_t>& guideWritten,
    RNG& rng,
    PhotonMapBuilder* photonMapper) {
    const int W = camera->imgWidth;
    const int H = camera->imgHeight;
    Color* outAlbedo = image.GetAlbedo();
    Vec3f* outNormal = image.GetNormal();
    auto StoreGuide = [&](int idx, const PathSampleResult& pr) {
        if (!pr.hit || guideWritten[idx]) return;
        guideWritten[idx] = 1;
        if (outAlbedo) outAlbedo[idx] = pr.albedo;
        if (outNormal) outNormal[idx] = SafeNormalize(pr.normal);
    };

    // Precompute camera frustum and basis once per bucket (old impl did this)
    const float fovRad = camera->fov * 3.14159265f / 180.0f;
    const float halfY = tan(0.5f * fovRad);
    const float aspect = (H > 0) ? float(W) / float(H) : 1.0f;
    const float halfX = halfY * aspect;

    const Vec3f f = camera->dir;
    const Vec3f r = (f.Cross(camera->up)).GetNormalized();
    const Vec3f u = (r.Cross(f)).GetNormalized();

    int rendered = 0;

    // Helper lambda: generate a ray for given sample position and index, with DOF
    auto GenerateRay = [&](float samplePosX, float samplePosY, int sampleIndex) -> Ray {
        const float ndcY = 1.0f - (samplePosY / float(H)) * 2.0f;
        const float sy = ndcY * halfY;
        const float ndcX = (samplePosX / float(W)) * 2.0f - 1.0f;
        const float sx = ndcX * halfX;

        Vec3f dirVec = r * sx + u * sy + f; // direction vector towards pixel (not normalized)

        Ray ray;
        if (camera->dof > 1e-6f) {
            // Thin lens model: compute point on focal plane, then sample lens disk
            float focus = camera->focaldist;
            float aperture = camera->dof; // treat 'dof' as lens radius

            Vec3f pixelWorld = camera->pos + dirVec * focus; // point on focus plane

            // Disk sampling using Halton with bases 5 and 7 and inverse-CDF for radius
            float hx = Halton(sampleIndex, 5);
            float hy = Halton(sampleIndex, 7);
            float rd = sqrtf(hx);
            float theta = 2.0f * float(M_PI) * hy;
            float dx = rd * cosf(theta);
            float dy = rd * sinf(theta);

            Vec3f lensPoint = camera->pos + r * (dx * aperture) + u * (dy * aperture);
            ray.p = lensPoint;
            ray.dir = (pixelWorld - lensPoint).GetNormalized();
        }
        else {
            ray.p = camera->pos;
            ray.dir = dirVec.GetNormalized();
        }
        return ray;
        };

    // Per-pixel accumulators for variance computation
    std::vector<int> pixelSamples(W * H, 0);
    std::vector<float> pixelSumR(W * H, 0.0f);
    std::vector<float> pixelSumG(W * H, 0.0f);
    std::vector<float> pixelSumB(W * H, 0.0f);
    std::vector<float> pixelSumR2(W * H, 0.0f);
    std::vector<float> pixelSumG2(W * H, 0.0f);
    std::vector<float> pixelSumB2(W * H, 0.0f);

    // Initial sampling phase
    for (int y = b.y0; y < b.y1 && !stopFlag; ++y) {
        for (int x = b.x0; x < b.x1; ++x) {
            // per-pixel Cranley-Patterson shifts
            float shiftX, shiftY;
            PixelShifts(x, y, shiftX, shiftY);

            float bestZ = BIGFLOAT;
            int origIdx = y * W + x; // index of the originating pixel
            int initial = std::max(kMinSamples, kInitSamples);
            for (int s = 0; s < initial; ++s) {
                int sampleIndex = (y * W + x) * kMaxSamples + (s + 1);
                float hx = Halton(sampleIndex, 2);
                float hy = Halton(sampleIndex, 3);
                // Cranley-Patterson rotation: add uniform shift and wrap
                float jitterX = fmodf(hx + shiftX, 1.0f) - 0.5f;
                float jitterY = fmodf(hy + shiftY, 1.0f) - 0.5f;

                // Subpixel position in image space
                float samplePosX = float(x) + 0.5f + jitterX;
                float samplePosY = float(y) + 0.5f + jitterY;

                // Generate ray (with DOF if enabled)
                Ray ray = GenerateRay(samplePosX, samplePosY, sampleIndex);

                PathSampleResult pr = TraceRadiance(*scene, *camera, ray, rng, x, y, s, photonMapper);
                Color sampleColor = pr.radiance;
                if (pr.primaryDist < bestZ) bestZ = pr.primaryDist;

                // Count this sample as originating from the original pixel
                originSampleCount[origIdx] += 1;
                StoreGuide(origIdx, pr);

                // Determine effective filter radius (use user override if provided)
                float radius = (g_filterRadius > 0.0f) ? g_filterRadius : FilterDefaultSupport(g_filterType);

                // Splat sample to nearby pixels using chosen filter (splatting)
                int x0 = std::max(0, int(std::floor(samplePosX - radius)));
                int x1 = std::min(W - 1, int(std::ceil(samplePosX + radius)));
                int y0 = std::max(0, int(std::floor(samplePosY - radius)));
                int y1 = std::min(H - 1, int(std::ceil(samplePosY + radius)));

                for (int yy = y0; yy <= y1; ++yy) {
                    float wy = Filter1D(float(yy) + 0.5f - samplePosY);
                    if (wy <= 0.0f) continue;
                    for (int xx = x0; xx <= x1; ++xx) {
                        float wx = Filter1D(float(xx) + 0.5f - samplePosX);
                        if (wx <= 0.0f) continue;
                        float w = wx * wy;
                        int idx = yy * W + xx;
                        accumColor[idx].r += sampleColor.r * w;
                        accumColor[idx].g += sampleColor.g * w;
                        accumColor[idx].b += sampleColor.b * w;
                        accumWeight[idx] += w;
                        if (bestZ < accumZ[idx]) accumZ[idx] = bestZ;
                    }
                }

                // Update per-pixel accumulators for variance
                pixelSamples[origIdx] += 1;
                pixelSumR[origIdx] += sampleColor.r;
                pixelSumG[origIdx] += sampleColor.g;
                pixelSumB[origIdx] += sampleColor.b;
                pixelSumR2[origIdx] += sampleColor.r * sampleColor.r;
                pixelSumG2[origIdx] += sampleColor.g * sampleColor.g;
                pixelSumB2[origIdx] += sampleColor.b * sampleColor.b;
            }
        }
    }

    // Compute high variance pixels
    std::vector<bool> hasHighVariance(W * H, false);
    for (int y = b.y0; y < b.y1 && !stopFlag; ++y) {
        for (int x = b.x0; x < b.x1; ++x) {
            int idx = y * W + x;
            int n = pixelSamples[idx];
            if (n <= 1) continue;
            float meanR = pixelSumR[idx] / float(n);
            float meanG = pixelSumG[idx] / float(n);
            float meanB = pixelSumB[idx] / float(n);
            float varR = pixelSumR2[idx] / float(n) - meanR * meanR;
            float varG = pixelSumG2[idx] / float(n) - meanG * meanG;
            float varB = pixelSumB2[idx] / float(n) - meanB * meanB;
            float vmax = std::max({ varR, varG, varB });
            if (vmax < 0.0f) vmax = 0.0f;
            float stddev = sqrtf(vmax);
            int df = n - 1;
            float t;
            if (df <= 3) t = 3.182f;
            else if (df <= 7) t = 2.365f;
            else if (df <= 15) t = 2.131f;
            else if (df <= 30) t = 2.042f;
            else t = 1.96f;
            float half_width = t * stddev / sqrtf(float(n));
            if (half_width > kMaxHalfWidth) {
                hasHighVariance[idx] = true;
            }
        }
    }

    // Combine statistical tests into needsRefinement (declare once here)
    std::vector<bool> needsRefinement(W * H, false);
    for (int y = b.y0; y < b.y1; ++y) {
        for (int x = b.x0; x < b.x1; ++x) {
            int idx = y * W + x;
            if (hasHighVariance[idx]) needsRefinement[idx] = true;
        }
    }

    // Adaptive refinement phase
    for (int y = b.y0; y < b.y1; ++y) {
        for (int x = b.x0; x < b.x1; ++x) {
            int idx = y * W + x;
            if (!needsRefinement[idx]) continue;

            // per-pixel Cranley-Patterson shifts
            float shiftX, shiftY;
            PixelShifts(x, y, shiftX, shiftY);

            // Adaptive sampling accumulators for this sample stream
            float bestZ = BIGFLOAT;
            int samplesTaken = pixelSamples[idx];

            while (samplesTaken < kMaxSamples) {
                int toAdd = std::min(kMinSamples, kMaxSamples - samplesTaken);
                bool added = false;
                for (int i = 0; i < toAdd; ++i) {
                    int s = samplesTaken;
                    int sampleIndex = (y * W + x) * kMaxSamples + (s + 1);
                    float hx = Halton(sampleIndex, 2);
                    float hy = Halton(sampleIndex, 3);
                    float jitterX = fmodf(hx + shiftX, 1.0f) - 0.5f;
                    float jitterY = fmodf(hy + shiftY, 1.0f) - 0.5f;

                    float samplePosX = float(x) + 0.5f + jitterX;
                    float samplePosY = float(y) + 0.5f + jitterY;

                    // Generate ray (with DOF if enabled)
                    Ray ray = GenerateRay(samplePosX, samplePosY, sampleIndex);

                    PathSampleResult pr = TraceRadiance(*scene, *camera, ray, rng, x, y, s, photonMapper);
                    Color sampleColor = pr.radiance;
                    if (pr.primaryDist < bestZ) bestZ = pr.primaryDist;

                    // Count this adaptive sample as originating from the original pixel
                    originSampleCount[idx] += 1;
                    StoreGuide(idx, pr);

                    float radius = (g_filterRadius > 0.0f) ? g_filterRadius : FilterDefaultSupport(g_filterType);

                    int x0 = std::max(0, int(std::floor(samplePosX - radius)));
                    int x1 = std::min(W - 1, int(std::ceil(samplePosX + radius)));
                    int y0 = std::max(0, int(std::floor(samplePosY - radius)));
                    int y1 = std::min(H - 1, int(std::ceil(samplePosY + radius)));

                    for (int yy = y0; yy <= y1; ++yy) {
                        float wy = Filter1D(float(yy) + 0.5f - samplePosY);
                        if (wy <= 0.0f) continue;
                        for (int xx = x0; xx <= x1; ++xx) {
                            float wx = Filter1D(float(xx) + 0.5f - samplePosX);
                            if (wx <= 0.0f) continue;
                            float w = wx * wy;
                            int nidx = yy * W + xx;
                            accumColor[nidx].r += sampleColor.r * w;
                            accumColor[nidx].g += sampleColor.g * w;
                            accumColor[nidx].b += sampleColor.b * w;
                            accumWeight[nidx] += w;
                            if (bestZ < accumZ[nidx]) accumZ[nidx] = bestZ;
                        }
                    }

                    // Update per-pixel accumulators
                    pixelSamples[idx] += 1;
                    pixelSumR[idx] += sampleColor.r;
                    pixelSumG[idx] += sampleColor.g;
                    pixelSumB[idx] += sampleColor.b;
                    pixelSumR2[idx] += sampleColor.r * sampleColor.r;
                    pixelSumG2[idx] += sampleColor.g * sampleColor.g;
                    pixelSumB2[idx] += sampleColor.b * sampleColor.b;
                    ++samplesTaken;
                    added = true;
                }
                if (!added) break;
                // Check if still needs more samples
                int n = pixelSamples[idx];
                float meanR = pixelSumR[idx] / float(n);
                float meanG = pixelSumG[idx] / float(n);
                float meanB = pixelSumB[idx] / float(n);
                float varR = pixelSumR2[idx] / float(n) - meanR * meanR;
                float varG = pixelSumG2[idx] / float(n) - meanG * meanG;
                float varB = pixelSumB2[idx] / float(n) - meanB * meanB;
                float vmax = std::max({ varR, varG, varB });
                if (vmax < 0.0f) vmax = 0.0f;
                float stddev = sqrtf(vmax);
                int df = n - 1;
                float t;
                if (df <= 3) t = 3.182f;
                else if (df <= 7) t = 2.365f;
                else if (df <= 15) t = 2.131f;
                else if (df <= 30) t = 2.042f;
                else t = 1.96f;
                float half_width = t * stddev / sqrtf(float(n));
                if (half_width <= kMaxHalfWidth) break;
            }

            ++rendered;
        }
    }

    // Compute provisional final colors from accumulators (for the bucket region)
    std::vector<Color> provisionalColor(W * H, Color(0, 0, 0));
    for (int y = b.y0; y < b.y1 && !stopFlag; ++y) {
        for (int x = b.x0; x < b.x1; ++x) {
            int idx = y * W + x;
            if (accumWeight[idx] > 0.0f) {
                provisionalColor[idx].r = accumColor[idx].r / accumWeight[idx];
                provisionalColor[idx].g = accumColor[idx].g / accumWeight[idx];
                provisionalColor[idx].b = accumColor[idx].b / accumWeight[idx];
            }
        }
    }

    // Count how many pixels in this bucket were actually rendered (have weight > 0)
    int pixelsRendered = 0;

    // Immediately write provisional results for this bucket into the shared render image
    {
        Color24* outPixels = image.GetPixels();
        float* outZ = image.GetZBuffer();
        int* outSamples = image.GetSampleCount();
        Color* outLinear = image.GetLinearPixels();
        for (int y = b.y0; y < b.y1; ++y) {
            for (int x = b.x0; x < b.x1; ++x) {
                int idx = y * W + x;
                if (accumWeight[idx] > 0.0f) {
                    Color finalCol = provisionalColor[idx];
                    if (outLinear) outLinear[idx] = finalCol;
                    outPixels[idx] = ToColor24(finalCol, camera->sRGB);
                    outZ[idx] = accumZ[idx];
                    outSamples[idx] = originSampleCount[idx];
                    ++pixelsRendered;
                }
            }
        }
    }

    // Notify the render image that pixels have been completed so viewport updates
    if (pixelsRendered > 0) {
        image.IncrementNumRenderPixel(pixelsRendered);
    }
}

void RayCaster::RenderProgressiveBucket(RenderImage& image, const Bucket& b,
    std::vector<Color>& accumColor,
    std::vector<float>& accumWeight,
    std::vector<int>& originSampleCount,
    std::vector<float>& accumZ,
    std::vector<uint8_t>& guideWritten,
    std::vector<int>& pixelSampleCount,
    RNG& rng,
    PhotonMapBuilder* photonMapper) {
    const int W = camera->imgWidth;
    const int H = camera->imgHeight;
    Color* outAlbedo = image.GetAlbedo();
    Vec3f* outNormal = image.GetNormal();
    auto StoreGuide = [&](int idx, const PathSampleResult& pr) {
        if (!pr.hit || guideWritten[idx]) return;
        guideWritten[idx] = 1;
        if (outAlbedo) outAlbedo[idx] = pr.albedo;
        if (outNormal) outNormal[idx] = SafeNormalize(pr.normal);
    };

    const float fovRad = camera->fov * 3.14159265f / 180.0f;
    const float halfY = tan(0.5f * fovRad);
    const float aspect = (H > 0) ? float(W) / float(H) : 1.0f;
    const float halfX = halfY * aspect;

    const Vec3f f = camera->dir;
    const Vec3f r = (f.Cross(camera->up)).GetNormalized();
    const Vec3f u = (r.Cross(f)).GetNormalized();

    auto GenerateRay = [&](float samplePosX, float samplePosY, int sampleIndex) -> Ray {
        const float ndcY = 1.0f - (samplePosY / float(H)) * 2.0f;
        const float sy = ndcY * halfY;
        const float ndcX = (samplePosX / float(W)) * 2.0f - 1.0f;
        const float sx = ndcX * halfX;

        Vec3f dirVec = r * sx + u * sy + f; // direction vector towards pixel (not normalized)

        Ray ray;
        if (camera->dof > 1e-6f) {
            // Thin lens model: compute point on focal plane, then sample lens disk
            float focus = camera->focaldist;
            float aperture = camera->dof; // treat 'dof' as lens radius

            Vec3f pixelWorld = camera->pos + dirVec * focus; // point on focus plane

            float hx = Halton(sampleIndex, 5);
            float hy = Halton(sampleIndex, 7);
            float rd = sqrtf(hx);
            float theta = 2.0f * float(M_PI) * hy;
            float dx = rd * cosf(theta);
            float dy = rd * sinf(theta);

            Vec3f lensPoint = camera->pos + r * (dx * aperture) + u * (dy * aperture);
            ray.p = lensPoint;
            ray.dir = (pixelWorld - lensPoint).GetNormalized();
        }
        else {
            ray.p = camera->pos;
            ray.dir = dirVec.GetNormalized();
        }
        return ray;
        };

    for (int y = b.y0; y < b.y1 && !stopFlag; ++y) {
        for (int x = b.x0; x < b.x1; ++x) {
            float shiftX, shiftY;
            PixelShifts(x, y, shiftX, shiftY);
            float bestZ = BIGFLOAT;
            int idx = y * W + x;

            for (int s = 0; s < kProgressiveSamplesPerPass; ++s) {
                int currentSamples = pixelSampleCount[idx];
                if (currentSamples >= kMaxSamples) break;

                int sampleIndex = (y * W + x) * kMaxSamples + (currentSamples + 1);
                pixelSampleCount[idx] = currentSamples + 1;

                float hx = Halton(sampleIndex, 2);
                float hy = Halton(sampleIndex, 3);
                float jitterX = fmodf(hx + shiftX, 1.0f) - 0.5f;
                float jitterY = fmodf(hy + shiftY, 1.0f) - 0.5f;

                float samplePosX = float(x) + 0.5f + jitterX;
                float samplePosY = float(y) + 0.5f + jitterY;

                Ray ray = GenerateRay(samplePosX, samplePosY, sampleIndex);

                PathSampleResult pr = TraceRadiance(*scene, *camera, ray, rng, x, y, currentSamples, photonMapper);
                Color sampleColor = pr.radiance;
                if (pr.primaryDist < bestZ) bestZ = pr.primaryDist;

                originSampleCount[idx] += 1;
                StoreGuide(idx, pr);

                float radius = (g_filterRadius > 0.0f) ? g_filterRadius : FilterDefaultSupport(g_filterType);

                int x0 = std::max(0, int(std::floor(samplePosX - radius)));
                int x1 = std::min(W - 1, int(std::ceil(samplePosX + radius)));
                int y0 = std::max(0, int(std::floor(samplePosY - radius)));
                int y1 = std::min(H - 1, int(std::ceil(samplePosY + radius)));

                for (int yy = y0; yy <= y1; ++yy) {
                    float wy = Filter1D(float(yy) + 0.5f - samplePosY);
                    if (wy <= 0.0f) continue;
                    for (int xx = x0; xx <= x1; ++xx) {
                        float wx = Filter1D(float(xx) + 0.5f - samplePosX);
                        if (wx <= 0.0f) continue;
                        float w = wx * wy;
                        int nidx = yy * W + xx;
                        accumColor[nidx].r += sampleColor.r * w;
                        accumColor[nidx].g += sampleColor.g * w;
                        accumColor[nidx].b += sampleColor.b * w;
                        accumWeight[nidx] += w;
                        if (bestZ < accumZ[nidx]) accumZ[nidx] = bestZ;
                    }
                }
            }
        }
    }
}

void RayCaster::CastRays(RenderImage& image, PhotonMapBuilder* photonMapper) {
    std::cout << "CastRays() entered; camera ptr=" << (void*)camera << "\n"; std::cout.flush();
    const int W = camera->imgWidth;
    const int H = camera->imgHeight;
    std::cout << "CastRays: image size W=" << W << " H=" << H << "\n"; std::cout.flush();
    std::cout << "Shading mode: " << (ShadingConfig::USE_LEGACY_SHADING ? "legacy (ShadeInfo)" : "path tracing") << "\n"; std::cout.flush();
    const int bucketSize = 16;
    buckets.clear();
    for (int y = 0; y < H; y += bucketSize) {
        for (int x = 0; x < W; x += bucketSize) {
            buckets.push_back(Bucket{ x, y, std::min(x + bucketSize, W), std::min(y + bucketSize, H) });
        }
    }
    nextBucket = 0;
    stopFlag = false;
    int nThreads = std::thread::hardware_concurrency();
    if (nThreads < 1) nThreads = 4;

    std::cout << "RayCaster: buckets=" << buckets.size() << " threads=" << nThreads << "\n";
    std::cout.flush();

    // Per-thread accumulation buffers to avoid atomics
    std::vector<std::vector<Color>> threadAccumColor(nThreads, std::vector<Color>(W * H, Color(0, 0, 0)));
    std::vector<std::vector<float>> threadAccumWeight(nThreads, std::vector<float>(W * H, 0.0f));
    std::vector<std::vector<int>>   threadOriginCount(nThreads, std::vector<int>(W * H, 0));
    std::vector<std::vector<float>> threadAccumZ(nThreads, std::vector<float>(W * H, BIGFLOAT));
    std::vector<std::vector<int>>   threadSampleCount;
    if (progressivePreview) {
        threadSampleCount.assign(nThreads, std::vector<int>(W * H, 0));
    }
    std::vector<uint8_t> guideWritten(W * H, 0);

    std::vector<RNG> threadRngs(nThreads);
    for (int t = 0; t < nThreads; ++t) {
        threadRngs[t].SetSequence(0x9e3779b97f4a7c15ULL + uint64_t(t));
    }
    auto mergeToImage = [&]() {
        Color24* outPixels = image.GetPixels();
        float* outZ = image.GetZBuffer();
        int* outSamples = image.GetSampleCount();
        Color* outLinear = image.GetLinearPixels();

        for (int i = 0; i < W * H; ++i) {
            float totalW = 0.0f;
            Color totalC(0, 0, 0);
            float minZ = BIGFLOAT;
            int totalOriginSamples = 0;
            for (int t = 0; t < nThreads; ++t) {
                totalW += threadAccumWeight[t][i];
                totalC.r += threadAccumColor[t][i].r;
                totalC.g += threadAccumColor[t][i].g;
                totalC.b += threadAccumColor[t][i].b;
                if (threadAccumZ[t][i] < minZ) minZ = threadAccumZ[t][i];
                totalOriginSamples += threadOriginCount[t][i];
            }
            Color final = Color(0, 0, 0);
            if (totalW > 0.0f) {
                final = Color(totalC.r / totalW, totalC.g / totalW, totalC.b / totalW);
            }
            else {
                final = Color(0, 0, 0);
            }
            if (outLinear) outLinear[i] = final;
            outPixels[i] = ToColor24(final, camera->sRGB);
            outZ[i] = minZ;
            outSamples[i] = totalOriginSamples;
        }
    };

    if (progressivePreview) {
        bool allDone = false;
        image.ResetNumRenderedPixels();
        bool seededCounter = false;
        size_t bucketCursor = 0;
        const size_t batchSize = std::max<size_t>(1, buckets.size() / 8);

        while (!stopFlag && !allDone) {
            size_t batchEnd = std::min(bucketCursor + batchSize, buckets.size());

            std::vector<std::thread> workers;
            workers.reserve(nThreads);
            for (int t = 0; t < nThreads; ++t) {
                workers.emplace_back([&, t]() {
                    try {
                        for (size_t b = bucketCursor + t; b < batchEnd && !stopFlag; b += nThreads) {
                    RenderProgressiveBucket(image, buckets[b],
                        threadAccumColor[t],
                        threadAccumWeight[t],
                        threadOriginCount[t],
                        threadAccumZ[t],
                        guideWritten,
                        threadSampleCount[t],
                        threadRngs[t],
                        photonMapper);
                        }
                    }
                    catch (const std::exception& e) {
                        std::cerr << "Worker " << t << " exception: " << e.what() << "\n";
                    }
                    catch (...) {
                        std::cerr << "Worker " << t << " unknown exception\n";
                    }
                    });
            }

            for (auto& th : workers) th.join();

            mergeToImage();
            if (!seededCounter) {
                image.IncrementNumRenderPixel(W * H);
                seededCounter = true;
            } else {
                image.IncrementNumRenderPixel(1);
            }
            if (stopFlag) break;

            allDone = true;
            int* outSamples = image.GetSampleCount();
            for (int i = 0; i < W * H; ++i) {
                if (outSamples[i] < kMaxSamples) {
                    allDone = false;
                    break;
                }
            }

            if (batchEnd >= buckets.size()) {
                bucketCursor = 0;
            } else {
                bucketCursor = batchEnd;
            }
        }

        return;
    }

    std::vector<std::thread> workers;
    workers.reserve(nThreads);
    for (int t = 0; t < nThreads; ++t) {
        workers.emplace_back([this, &image, t, &threadAccumColor, &threadAccumWeight, &threadOriginCount, &threadAccumZ,
                              &guideWritten, &threadRngs, photonMapper]() {
            try {
                for (;;) {
                    size_t b = nextBucket.fetch_add(1, std::memory_order_relaxed);
                    if (b >= buckets.size()) break;
                    if (stopFlag) break;
                    RenderBucket(image, buckets[b],
                        threadAccumColor[t],
                        threadAccumWeight[t],
                        threadOriginCount[t],
                        threadAccumZ[t],
                        guideWritten,
                        threadRngs[t],
                        photonMapper);
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Worker " << t << " exception: " << e.what() << "\n";
            }
            catch (...) {
                std::cerr << "Worker " << t << " unknown exception\n";
            }
            });
    }

    for (auto& th : workers) th.join();

    mergeToImage();
    image.ComputeZBufferImage();
}

void RayCaster::Stop() {
    stopFlag = true;
}

// RayCastRenderer implementations
void RayCastRenderer::BeginRender() {
    if (!raycaster) return;
    isRendering = true;
    // launch background thread that runs CastRays
    renderThread = std::thread([this]() {
        try {
            auto t0 = std::chrono::steady_clock::now();
            raycaster->CastRays(this->GetRenderImage(), photonMapper);
            auto t1 = std::chrono::steady_clock::now();
            const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            const double total_seconds = ms / 1000.0;
            const int hours = static_cast<int>(total_seconds) / 3600;
            const int minutes = (static_cast<int>(total_seconds) % 3600) / 60;
            const int seconds = static_cast<int>(total_seconds) % 60;
            std::cout << "Render time: " << std::setfill('0') << std::setw(2) << hours << ":"
                << std::setfill('0') << std::setw(2) << minutes << ":"
                << std::setfill('0') << std::setw(2) << seconds << "\n";

            if (!isPreviewMode) {
                // Save outputs when finished (unless disabled by caller)
                if (autoSaveOutputs) {
                    this->GetRenderImage().ComputeZBufferImage();
                    this->GetRenderImage().ComputeSampleCountImage();
                    this->GetRenderImage().SaveImage(outputColorPath.c_str());
                    this->GetRenderImage().SaveZImage(outputZPath.c_str());
                    this->GetRenderImage().SaveSampleCountImage(outputSamplesPath.c_str());
                }

                // Mark rendering as complete
                isRendering = false;
            } else {
                if (denoiseMode != Denoiser::DenoiseMode::None) {
                    Color24* pixels = this->GetRenderImage().GetPixels();
                    std::string err;
                    bool ok = false;
                    switch (denoiseMode) {
                    case Denoiser::DenoiseMode::MedianOnly:
                        ok = Denoiser::ApplyMedian(this->GetRenderImage(), this->GetCamera().sRGB, std::string(), pixels, &err);
                        break;
                    case Denoiser::DenoiseMode::OIDNOnly:
                        if (Denoiser::IsOIDNAvailable()) {
                            ok = Denoiser::ApplyOIDN(this->GetRenderImage(), this->GetCamera().sRGB, std::string(), pixels, &err);
                        } else {
                            err = "OIDN denoiser not available in this build.";
                        }
                        break;
                    case Denoiser::DenoiseMode::MedianThenOIDN:
                        if (Denoiser::IsOIDNAvailable()) {
                            ok = Denoiser::ApplyMedianThenOIDN(this->GetRenderImage(), this->GetCamera().sRGB, std::string(), pixels, &err);
                        } else {
                            ok = Denoiser::ApplyMedian(this->GetRenderImage(), this->GetCamera().sRGB, std::string(), pixels, &err);
                        }
                        break;
                    default:
                        break;
                    }
                    if (!ok && !err.empty()) {
                        std::cerr << "Preview denoise failed: " << err << "\n";
                    }
                    // Force a viewport refresh after denoise by nudging the rendered pixel counter.
                    // In preview mode the GLUT idle loop only repaints when this counter changes.
                    RenderImage& img = this->GetRenderImage();
                    const int totalPixels = img.GetWidth() * img.GetHeight();
                    if (img.GetNumRenderedPixels() >= totalPixels) {
                        img.IncrementNumRenderPixel(1);
                    }
                }
            }
            // For preview mode, keep isRendering = true so viewport stays open
        } catch (const std::exception& e) {
            std::cerr << "Render thread exception: " << e.what() << "\n";
            isRendering = false;
        } catch (...) {
            std::cerr << "Render thread unknown exception.\n";
            isRendering = false;
        }
        });
}

void RayCastRenderer::StopRender() {
    if (raycaster) raycaster->Stop();
    if (renderThread.joinable()) renderThread.join();
    isRendering = false;
}

bool RayCastRenderer::TraceRay(Ray const& ray, HitInfo& hInfo, int hitSide) const {
    // Forward to scene intersection
    hInfo.Init();
    return IntersectNode(&GetScene().rootNode, ray, hInfo, hitSide);
}

bool RayCastRenderer::TraceShadowRay(Ray const& ray, float t_max, int hitSide) const {
    HitInfo h;
    if (IntersectNode(&GetScene().rootNode, ray, h, hitSide)) {
        float d = (h.p - ray.p).Length();
        if (d < t_max) {
            // For simplicity, assume no absorption here; full occlusion
            return 0.0f;
        }
    }
    return 1.0f; // Fully visible
}

PhotonMap const* RayCastRenderer::GetPhotonMap() const {
    return photonMapper ? photonMapper->GetPhotonMap() : nullptr;
}

PhotonMap const* RayCastRenderer::GetCausticsMap() const {
    return photonMapper ? photonMapper->GetCausticsMap() : nullptr;
}
