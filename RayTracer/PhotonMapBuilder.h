#pragma once

#include "ShadingConfig.h"
#include "scene.h"
#include "photonmap.h"
#include "rng.h"
#include <vector>
#include <atomic>

// PhotonMapBuilder: Handles photon emission, tracing, and photon map construction
// Does NOT perform pixel rendering - that's handled by RayCaster
class PhotonMapBuilder
{
public:
    PhotonMapBuilder();
    virtual ~PhotonMapBuilder();

    // Build photon maps from a scene
    void BuildPhotonMap(Scene* scene);

    // Access to photon maps
    PhotonMap const* GetPhotonMap() const { return &photonMap; }
    PhotonMap const* GetCausticsMap() const { return &causticsMap; }

    // Photon mapping parameters
    void SetNumPhotons(int n) { numPhotons = n; }
    void SetPhotonSearchRadius(float r) {
        photonSearchRadius = r;
        photonRadiusOverridden = true;
        if (!causticRadiusOverridden) {
            causticSearchRadius = r * 0.25f;
            if (causticSearchRadius < 0.01f) causticSearchRadius = 0.01f;
        }
    }
    void SetCausticSearchRadius(float r) { causticSearchRadius = r; causticRadiusOverridden = true; }
    void SetPhotonSearchCount(int c) { photonSearchCount = c; }
    float GetPhotonSearchRadius() const { return photonSearchRadius; }
    float GetCausticSearchRadius() const { return causticSearchRadius; }
    int GetPhotonSearchCount() const { return photonSearchCount; }

private:
    PhotonMap photonMap;       // Global photon map - stores diffuse photons
    PhotonMap causticsMap;     // Caustics map - stores caustic photons (S+D paths)

    // Photon mapping parameters
    int numPhotons = 10000000;      // Total number of photons to emit
    float photonSearchRadius = 0.2f;    // Search radius for indirect/global photons
    float causticSearchRadius = 0.05f;   // Separate radius for caustic lookups
    int photonSearchCount = 100;// Number of photons to gather
    bool photonRadiusOverridden = false;
    bool causticRadiusOverridden = false;

    // Photon tracing
    int maxBounces = 2;  // Maximum number of bounces for photon tracing
    void EmitPhotons(Scene* scene);
    void TracePhoton(Scene* scene, RNG& rng, Ray const& ray, Color const& power, int bounce, bool causticPath, bool hasHitDiffuse, bool hasTransmitted);

    // Helper functions
    void AdjustPhotonGatherSettings(Scene* scene);

    // Debug counters (updated during photon emission)
    std::atomic<long long> debugTraceCount{0};
    std::atomic<long long> debugStoreCount{0};
};
