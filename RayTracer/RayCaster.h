#pragma once

#include <vector>
#include <thread>
#include <atomic>
#include <cstdint>
#include <string>
#include "ShadingConfig.h"
#include "renderer.h"
#include "scene.h"
#include "objects.h"
#include "rng.h"
#include <memory>
#include "Denoiser.h"

// Forward declaration
class PhotonMapBuilder;

struct Bucket {
    int x0, y0, x1, y1;
};

enum class FilterType { Tent, Gaussian, Mitchell, Lanczos };

// Global filter selection (set from main)
extern FilterType g_filterType;
extern float      g_filterRadius;

class RayCaster {
public:
    RayCaster(Scene* scene, Camera* camera);
    void CastRays(RenderImage& image, PhotonMapBuilder* photonMapper = nullptr);
    void Stop();
    bool IsStopped() const { return stopFlag.load(); }
    void SetProgressivePreview(bool progressive) { progressivePreview = progressive; }
private:
    Scene* scene;
    Camera* camera;
    std::vector<Bucket> buckets;
    std::atomic<size_t> nextBucket{ 0 };
    // RenderBucket now takes per-worker accumulation buffers for reconstruction filtering
    void RenderBucket(RenderImage& image, const Bucket& b,
        std::vector<Color>& accumColor,
        std::vector<float>& accumWeight,
        std::vector<int>& originSampleCount,
        std::vector<float>& accumZ,
        std::vector<uint8_t>& guideWritten,
        RNG& rng,
        PhotonMapBuilder* photonMapper);
    void RenderProgressiveBucket(RenderImage& image, const Bucket& b,
        std::vector<Color>& accumColor,
        std::vector<float>& accumWeight,
        std::vector<int>& originSampleCount,
        std::vector<float>& accumZ,
        std::vector<uint8_t>& guideWritten,
        std::vector<int>& pixelSampleCount,
        RNG& rng,
        PhotonMapBuilder* photonMapper);
    // Stop flag for cooperative cancelation
    std::atomic<bool> stopFlag{ false };
    bool progressivePreview = false;
};

// Renderer wrapper that runs RayCaster on background thread(s) and integrates
// with the viewport BeginRender/StopRender lifecycle.
class RayCastRenderer : public Renderer {
public:
    RayCastRenderer() = default;
    ~RayCastRenderer() { StopRender(); }

    bool LoadScene(char const* sceneFilename) override {
        if (!Renderer::LoadScene(sceneFilename)) return false;
        // initialize raycaster with scene & camera
        raycaster = std::make_unique<RayCaster>(&GetScene(), &GetCamera());
        raycaster->SetProgressivePreview(progressivePreview);
        return true;
    }

    void BeginRender() override;
    void StopRender() override;

    bool TraceRay(Ray const& ray, HitInfo& hInfo, int hitSide = HIT_FRONT_AND_BACK) const override;
    bool TraceShadowRay(Ray const& ray, float t_max, int hitSide = HIT_FRONT_AND_BACK) const override;
    PhotonMap const* GetPhotonMap() const override;
    PhotonMap const* GetCausticsMap() const override;

    // Set preview mode
    void SetPreviewMode(bool preview) { isPreviewMode = preview; }
    // Set progressive preview mode (only applies when preview is enabled)
    void SetProgressivePreview(bool progressive) { progressivePreview = progressive; if (raycaster) raycaster->SetProgressivePreview(progressive); }
    // Set runtime denoiser mode and kernel
    void SetDenoiseMode(Denoiser::DenoiseMode mode) { denoiseMode = mode; }
    // Control offline output filenames (default: render.png, zbuffer.png, samples.png, render_denoised.png)
    void SetOutputPaths(const std::string& color, const std::string& z, const std::string& samples, const std::string& denoised) {
        outputColorPath = color;
        outputZPath = z;
        outputSamplesPath = samples;
        outputDenoisedPath = denoised;
    }
    void SetAutoSaveOutputs(bool enable) { autoSaveOutputs = enable; }

    // Set photon mapper
    void SetPhotonMapper(PhotonMapBuilder* mapper) { photonMapper = mapper; }

private:
    std::unique_ptr<RayCaster> raycaster;
    std::thread renderThread;
    bool isPreviewMode = false;
    bool progressivePreview = false;
    Denoiser::DenoiseMode denoiseMode = Denoiser::DenoiseMode::None;
    PhotonMapBuilder* photonMapper = nullptr;
    std::string outputColorPath = "render.png";
    std::string outputZPath = "zbuffer.png";
    std::string outputSamplesPath = "samples.png";
    std::string outputDenoisedPath = "render_denoised.png";
    bool autoSaveOutputs = true;
};
