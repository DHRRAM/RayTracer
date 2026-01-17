#include "PhotonMapBuilder.h"
#include "IntersectNodeAdapter.h"
#include "BumpMapping.h"
#include "materials.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <thread>
#include <atomic>

PhotonMapBuilder::PhotonMapBuilder() {
    // Constructor
}

PhotonMapBuilder::~PhotonMapBuilder() {
    // Destructor
}

void PhotonMapBuilder::BuildPhotonMap(Scene* scene) {
    if (!scene) return;

    try {
        AdjustPhotonGatherSettings(scene);

        // Allocate space for photons
        photonMap.Resize(numPhotons);
        causticsMap.Resize(numPhotons / 10); // Smaller caustics map

        std::cout << "Building photon map with " << numPhotons << " photons...\n";
        std::cout << "Emitting photons from light sources...\n";
        EmitPhotons(scene);
        std::cout << "Finished emitting photons.\n";

        std::cout << "Photon map contains " << photonMap.NumPhotons() << " photons\n";
        std::cout << "Caustics map contains " << causticsMap.NumPhotons() << " photons\n";

        if (photonMap.NumPhotons() > 0) {
            // Scale by 1/N where N is the number of photons we tried to emit
            float scale = 1.0f / float(numPhotons);
            photonMap.ScalePhotonPowers(scale);
            photonMap.PrepareForIrradianceEstimation();
        }

        if (causticsMap.NumPhotons() > 0) {
            // Normalize by the number of stored caustic photons so their energy isn't under-weighted
            float scale = 1.0f / float(causticsMap.NumPhotons());
            causticsMap.ScalePhotonPowers(scale);
            causticsMap.PrepareForIrradianceEstimation();
        }

        std::cout << "Photon map build complete!\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Photon map build failed with exception: " << e.what() << "\n";
    }
    catch (...) {
        std::cerr << "Photon map build failed with unknown exception.\n";
    }
}

void PhotonMapBuilder::AdjustPhotonGatherSettings(Scene* scene) {
    const Box& bounds = scene->rootNode.GetChildBoundBox();
    if (bounds.IsEmpty()) return;

    Vec3f extent = bounds.pmax - bounds.pmin;
    float maxExtent = std::max({ fabsf(extent.x), fabsf(extent.y), fabsf(extent.z) });
    if (maxExtent <= 0.0f) return;

    if (!photonRadiusOverridden) {
        float recommended = std::max(0.05f, maxExtent * 0.02f);
        photonSearchRadius = recommended;
    }

    if (!causticRadiusOverridden) {
        causticSearchRadius = std::max(0.01f, photonSearchRadius * 0.25f);
    }
}

void PhotonMapBuilder::EmitPhotons(Scene* scene) {
    // Count total light intensity for importance sampling
    float totalIntensity = 0.0f;
    std::vector<float> lightIntensities;

    for (const Light* light : scene->lights) {
        if (light->IsPhotonSource()) {
            Color intensity = light->Intensity();
            float power = intensity.r + intensity.g + intensity.b;
            lightIntensities.push_back(power);
            totalIntensity += power;
        }
        else {
            lightIntensities.push_back(0.0f);
        }
    }

    if (totalIntensity <= 0.0f) {
        std::cout << "Warning: No photon light sources found!\n";
        return;
    }

    unsigned int numThreads = std::max(1u, std::thread::hardware_concurrency());
    std::atomic<int> photonCounter(0);
    std::vector<std::thread> workers;
    workers.reserve(numThreads);

    auto worker = [&](unsigned int threadIndex) {
        RNG threadRng(0x9e3779b97f4a7c15ULL + threadIndex);
        while (true) {
            int photonId = photonCounter.fetch_add(1, std::memory_order_relaxed);
            if (photonId >= numPhotons) break;

            float r = threadRng.RandomFloat() * totalIntensity;
            float sum = 0.0f;
            int lightIndex = 0;
            for (size_t j = 0; j < scene->lights.size(); ++j) {
                sum += lightIntensities[j];
                if (r <= sum) {
                    lightIndex = static_cast<int>(j);
                    break;
                }
            }

            const Light* light = scene->lights[lightIndex];
            if (!light->IsPhotonSource()) continue;

            Ray photonRay;
            Color photonPower;
            light->RandomPhoton(threadRng, photonRay, photonPower);

            float lightSelectionPdf = lightIntensities[lightIndex] / totalIntensity;
            Color scaledPower = photonPower * (1.0f / lightSelectionPdf);

            TracePhoton(scene, threadRng, photonRay, scaledPower, 0, false, false, false);
        }
    };

    // Launch workers
    for (unsigned int t = 0; t < numThreads; ++t) {
        workers.emplace_back(worker, t);
    }

    // Wait for completion
    for (auto& th : workers) {
        if (th.joinable()) th.join();
    }
}

void PhotonMapBuilder::TracePhoton(Scene* scene, RNG& rng, const Ray& ray, const Color& power, int bounce, bool causticPath, bool hasHitDiffuse, bool hasTransmitted) {

    if (bounce > maxBounces) return;
    if (power.Max() < 1e-6f) return;

    HitInfo hInfo;
    hInfo.Init();
    if (!IntersectNode(&scene->rootNode, ray, hInfo, HIT_FRONT_AND_BACK)) {
        return;
    }

    // Ignore back hits to prevent incorrect caustics
    //if (!hInfo.front) return;

    const Material* mtl = hInfo.node ? hInfo.node->GetMaterial() : nullptr;
    if (!mtl) return;
    ApplyBumpMapping(mtl, hInfo);

    Vec3f hitPos = hInfo.p;
    Vec3f N = hInfo.N;
    N.Normalize();

    // Determine the correct shading normal (facing the ray)
    bool inside = (ray.dir % N) > 0.0f;

    bool isDiffuseSurface = mtl->IsPhotonSurface(hInfo.mtlID);
    bool storeAsCaustic = causticPath && hasTransmitted && !hasHitDiffuse && isDiffuseSurface;

    // Store photon on diffuse surfaces after the first bounce
    if (bounce >= 1 && isDiffuseSurface) {
        if (storeAsCaustic) {
            causticsMap.AddPhoton(hitPos, ray.dir.GetNormalized(), power);
        }
        else {
            photonMap.AddPhoton(hitPos, ray.dir.GetNormalized(), power);
        }

        hasHitDiffuse = true;
        causticPath = false;
    }

    // ----- Use GenerateSample to choose next direction -----
    // Create a minimal SamplerInfo for BRDF sampling
    class PhotonSamplerInfo : public SamplerInfo {
    public:
        PhotonSamplerInfo(RNG& r, const HitInfo& h, const Ray& ray, bool isFront)
            : SamplerInfo(r), hInfo(h), viewDir(-ray.dir.GetNormalized()), frontFacing(isFront) {
        }

        Vec3f P()  const override { return hInfo.p; }
        Vec3f V()  const override { return viewDir; }
        Vec3f N()  const override { return hInfo.N; }
        Vec3f GN() const override { return hInfo.GN; }
        bool IsFront() const override { return frontFacing; }
        Vec3f UVW() const override { return hInfo.uvw; }
        Vec3f dUVW_dX() const override { return hInfo.duvw[0]; }
        Vec3f dUVW_dY() const override { return hInfo.duvw[1]; }
        int MaterialID() const override { return hInfo.mtlID; }

    private:
        HitInfo hInfo;
        Vec3f viewDir;
        bool frontFacing;
    };

    PhotonSamplerInfo samplerInfo(rng, hInfo, ray, !inside);
    Vec3f nextDir;
    DirSampler::Info si;

    if (!mtl->GenerateSample(samplerInfo, nextDir, si)) {
        return; // No valid sample generated
    }

    // If we hit a purely specular surface before any diffuse hit, treat it as a blocker
    // for the global photon map. Otherwise, photons can leak through reflective objects
    // and falsely light the surface right behind them (losing the contact shadow).
    if (!hasHitDiffuse && si.lobe == DirSampler::Lobe::SPECULAR && si.mult.Max() > 0.0f) {
        return;
    }

    // Apply Russian roulette for path termination
    Color adjustedPower = power;
    if (bounce >= 0) {
        float continueProbability = std::min(0.5f, si.mult.Max());
        if (continueProbability < 1e-6f) return;

        if (rng.RandomFloat() > continueProbability) return;
        adjustedPower = power * (1.0f / continueProbability);
    }

    // Scale power by BRDF and inverse probability
    Color nextPower = adjustedPower * si.mult;
    if (si.prob > 1e-6f) {
        nextPower = nextPower * (1.0f / si.prob);
    }
    else {
        return; // Invalid probability
    }

    // Determine if next path is caustic
    bool nextCaustic = causticPath;
    bool nextHasTransmitted = hasTransmitted;
    if (si.lobe == DirSampler::Lobe::DIFFUSE) {
        nextCaustic = false;
    }
    else if (si.lobe == DirSampler::Lobe::TRANSMISSION && !hasHitDiffuse) {
        // Only start a caustic path on refractive/transmissive events
        nextCaustic = true;
        nextHasTransmitted = true;
    }

    // Bias the next ray origin
    Vec3f Nn = inside ? -N : N;
    Vec3f biasDir = nextDir.GetNormalized();
    float biasAmount = 1e-4f;
    Vec3f nextOrigin = hitPos + ((Nn % biasDir) >= 0.0f ? Nn : -Nn) * biasAmount;

    Ray nextRay(nextOrigin, biasDir);
    TracePhoton(scene, rng, nextRay, nextPower, bounce + 1, nextCaustic, hasHitDiffuse, nextHasTransmitted);
}
