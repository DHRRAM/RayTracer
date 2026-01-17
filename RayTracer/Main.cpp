#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <memory>
#include <exception>
#include <csignal>
#include <filesystem>
#include <cstring>
#include <sstream>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "renderer.h"
#include "xmlload.h"
#include "RayCaster.h"
#include "BBoxConfig.h"
#include "BVHConfig.h"
#include "PhotonMapBuilder.h"
#include "PhotonMappingConfig.h"
#include "Denoiser.h"

bool g_rtUseBBox     = true; // default ON
bool g_rtUseMeshBVH  = true; // default ON
Denoiser::DenoiseMode g_rtDenoiseMode = Denoiser::DenoiseMode::None;

static void PrintUsage() {
    std::cout << "Usage: RayTracer <scene.xml> [--preview] [--progressive] [--bbox|--nobbox] [--bvh|--nobvh] [--filter=name] [--filter-radius=value] [--denoise[=mode]] [--nodenoise]\n";
    std::cout << "  filter names: tent, gaussian, mitchell, lanczos; --progressive only affects --preview renders\n";
    std::cout << "  --denoise enables denoising (default mode: median+oidn). Modes: oidn, median, median+oidn\n";
    std::cout << "  Animation: --time=t | --anim-start=a --anim-end=b [--anim-fps=f] [--anim-frames=n]\n";
    std::cout << "  Output names: --out-prefix=prefix (default: render)\n";
}

static void SaveSampleCountImageAndReport(RenderImage& img, const std::string& filename = "samples.png") {
    // Generate visualization image and save it
    int vmax = img.ComputeSampleCountImage();
    if (!img.SaveSampleCountImage(filename.c_str())) {
        std::cerr << "Failed to save " << filename << "\n";
    }
    else {
        // Compute exact min/max ignoring zero (unrendered)
        int* sc = img.GetSampleCount();
        int w = img.GetWidth();
        int h = img.GetHeight();
        int vmin = INT_MAX;
        int vmax_exact = 0;
        for (int i = 0; i < w * h; ++i) {
            int v = sc[i];
            if (v == 0) continue;
            if (v < vmin) vmin = v;
            if (v > vmax_exact) vmax_exact = v;
        }
        if (vmin == INT_MAX) vmin = 0; // nothing rendered
        std::cout << "Saved samples.png (visual vmax=" << vmax << ") -- samples per pixel: min=" << vmin << " max=" << vmax_exact << "\n";
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        PrintUsage();
        return 0;
    }

    const char* sceneFile = nullptr;
    bool preview = false;
    bool progressivePreview = false;
    double animStart = 0.0;
    double animEnd = 0.0;
    int animFrames = -1;
    double animFPS = 24.0;
    double singleTime = -1.0;
    bool forceAnimation = false;
    std::string outPrefix = "render";

    // ----- argument parsing -----
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--preview") {
            preview = true;
        }
        else if (arg == "--progressive") {
            progressivePreview = true;
        }
        else if (arg == "--bbox") {
            g_rtUseBBox = true;
        }
        else if (arg == "--nobbox") {
            g_rtUseBBox = false;
        }
        else if (arg == "--bvh") {
            g_rtUseMeshBVH = true;
        }
        else if (arg == "--nobvh") {
            g_rtUseMeshBVH = false;
        }
        else if (arg.rfind("--filter=", 0) == 0) {
            std::string val = arg.substr(strlen("--filter="));
            if (val == "tent") g_filterType = FilterType::Tent;
            else if (val == "gaussian") g_filterType = FilterType::Gaussian;
            else if (val == "mitchell") g_filterType = FilterType::Mitchell;
            else if (val == "lanczos") g_filterType = FilterType::Lanczos;
            else std::cerr << "Unknown filter: " << val << "\n";
        }
        else if (arg.rfind("--filter-radius=", 0) == 0) {
            std::string val = arg.substr(strlen("--filter-radius="));
            try {
                g_filterRadius = std::stof(val);
            }
            catch (...) {
                std::cerr << "Invalid filter radius: " << val << "\n";
            }
        }
        else if (arg.rfind("--time=", 0) == 0) {
            std::string val = arg.substr(strlen("--time="));
            try {
                singleTime = std::stod(val);
            }
            catch (...) {
                std::cerr << "Invalid time value: " << val << "\n";
            }
        }
        else if (arg.rfind("--anim-start=", 0) == 0) {
            std::string val = arg.substr(strlen("--anim-start="));
            try {
                animStart = std::stod(val);
                forceAnimation = true;
            }
            catch (...) {
                std::cerr << "Invalid anim start time: " << val << "\n";
            }
        }
        else if (arg.rfind("--anim-end=", 0) == 0) {
            std::string val = arg.substr(strlen("--anim-end="));
            try {
                animEnd = std::stod(val);
                forceAnimation = true;
            }
            catch (...) {
                std::cerr << "Invalid anim end time: " << val << "\n";
            }
        }
        else if (arg.rfind("--anim-fps=", 0) == 0) {
            std::string val = arg.substr(strlen("--anim-fps="));
            try {
                animFPS = std::stod(val);
            }
            catch (...) {
                std::cerr << "Invalid anim fps: " << val << "\n";
            }
        }
        else if (arg.rfind("--anim-frames=", 0) == 0) {
            std::string val = arg.substr(strlen("--anim-frames="));
            try {
                animFrames = std::stoi(val);
                forceAnimation = true;
            }
            catch (...) {
                std::cerr << "Invalid anim frame count: " << val << "\n";
            }
        }
        else if (arg.rfind("--out-prefix=", 0) == 0) {
            outPrefix = arg.substr(strlen("--out-prefix="));
        }
        else if (arg == "--nodenoise" || arg == "--nodenoiseruntime") {
            g_rtDenoiseMode = Denoiser::DenoiseMode::None;
        }
        else if (arg.rfind("--denoise", 0) == 0) {
            // Default to hybrid median+OIDN when denoise flag is provided
            g_rtDenoiseMode = Denoiser::DenoiseMode::MedianThenOIDN;
            if (arg != "--denoise") {
                std::string val = arg.substr(strlen("--denoise="));
                if (val == "oidn" || val == "oidnonly") {
                    g_rtDenoiseMode = Denoiser::DenoiseMode::OIDNOnly;
                }
                else if (val == "median" || val == "medianonly") {
                    g_rtDenoiseMode = Denoiser::DenoiseMode::MedianOnly;
                }
                else if (val == "median+oidn" || val == "median-oidn" || val == "hybrid") {
                    g_rtDenoiseMode = Denoiser::DenoiseMode::MedianThenOIDN;
                }
                else {
                    std::cerr << "Unknown denoise mode: " << val << " (use oidn, median, or median+oidn)\n";
                }
            }
        }
    else if (!arg.empty() && arg.rfind("--", 0) == std::string::npos) {
            sceneFile = argv[i];
        }
    }

    if (!sceneFile) {
        PrintUsage();
        return 0;
    }
    if (outPrefix.empty()) outPrefix = "render";

    // Emit process directory for sanity
    if (!preview && progressivePreview) {
        std::cout << "Ignoring --progressive because preview mode is not enabled.\n";
        progressivePreview = false;
    }

    std::unique_ptr<RayCastRenderer> renderer = std::make_unique<RayCastRenderer>();
    std::unique_ptr<PhotonMapBuilder> photonMapper;

    try {
        if (!renderer->LoadScene(sceneFile)) {
            std::cerr << "Failed to load scene: " << sceneFile << "\n";
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception during LoadScene: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception during LoadScene.\n";
        return 1;
    }
    renderer->SetDenoiseMode(g_rtDenoiseMode);

    // Build photon map if photon mapping is enabled
    if (PhotonMappingConfig::IsPhotonMappingEnabled()) {
        std::cout << "Photon mapping enabled - building photon maps...\n";
        photonMapper = std::make_unique<PhotonMapBuilder>();
        try {
            photonMapper->BuildPhotonMap(&renderer->GetScene());
        } catch (const std::exception& e) {
            std::cerr << "Exception while building photon map: " << e.what() << "\n";
            return 1;
        } catch (...) {
            std::cerr << "Unknown exception while building photon map.\n";
            return 1;
        }
        renderer->SetPhotonMapper(photonMapper.get());
    }
    else {
        std::cout << "Using standard ray tracing (no photon mapping)\n";
    }

    // Preview: let the viewport drive BeginRender/StopRender
    if (preview) {
        // Allow previewing a specific animation frame
        if (singleTime >= 0.0) {
            Camera& cameraRef = renderer->GetCamera();
            cameraRef.EvaluateAtTime(singleTime);
        }
        renderer->SetPreviewMode(true);
        renderer->SetProgressivePreview(progressivePreview);
        ShowViewport(renderer.get(), true);
        return 0;
    }

    renderer->SetAutoSaveOutputs(false);
    Camera& cameraRef = renderer->GetCamera();
    bool cameraHasAnim = cameraRef.HasAnimation();
    if (cameraHasAnim && animEnd <= animStart) {
        animEnd = cameraRef.GetAnimationEnd();
    }
    bool renderSequence = (singleTime < 0.0) && (forceAnimation || (cameraHasAnim && (animFrames != 0) && (animEnd > animStart || animFrames > 1)));
    if (renderSequence && animFPS <= 0.0) animFPS = 24.0;
    if (renderSequence && animFrames <= 0) {
        double duration = std::max(0.0, animEnd - animStart);
        animFrames = std::max(1, static_cast<int>(std::floor(duration * animFPS)) + 1);
    }
    if (renderSequence && animFrames == 1) renderSequence = false;

    if (g_rtDenoiseMode == Denoiser::DenoiseMode::None) {
        std::cout << "Denoising disabled by user; skipping denoised output.\n";
    }

    auto formatBaseNames = [&](const std::string& prefix, int frameIdx, int pad) {
        std::ostringstream oss;
        std::string base;
        std::string colorOut, zOut, samplesOut, denoisedOut;
        
        if (frameIdx >= 0) {
            // Animation sequence - organize into subdirectories
            std::string frameName = prefix + "_" + 
                std::string(pad - std::to_string(frameIdx).length(), '0') + std::to_string(frameIdx);
            
            // Create subdirectories if they don't exist
            std::filesystem::create_directories("beauty");
            std::filesystem::create_directories("zdepth");
            std::filesystem::create_directories("samples");
            std::filesystem::create_directories("denoised");
            
            colorOut = "beauty/" + frameName + ".png";
            zOut = "zdepth/" + frameName + "_z.png";
            samplesOut = "samples/" + frameName + "_samples.png";
            denoisedOut = "denoised/" + frameName + "_denoised.png";
        }
        else {
            // Single frame - use current directory with legacy names
            base = prefix;
            colorOut = base + ".png";
            zOut = (prefix == "render") ? "zbuffer.png" : base + "_z.png";
            samplesOut = (prefix == "render") ? "samples.png" : base + "_samples.png";
            denoisedOut = (prefix == "render") ? "render_denoised.png" : base + "_denoised.png";
        }
        
        return std::tuple<std::string, std::string, std::string, std::string>(colorOut, zOut, samplesOut, denoisedOut);
    };

    auto applyDenoiser = [&](const std::string& denoisedOut) {
        if (g_rtDenoiseMode == Denoiser::DenoiseMode::None) {
            std::cout << "Denoising disabled by user; skipping denoised output.\n";
            return;
        }
        std::string err;
        bool ok = false;
        switch (g_rtDenoiseMode) {
        case Denoiser::DenoiseMode::MedianOnly:
            ok = Denoiser::ApplyMedian(renderer->GetRenderImage(), cameraRef.sRGB, denoisedOut, nullptr, &err);
            if (ok) std::cout << "Saved median-filtered image to " << denoisedOut << "\n";
            else std::cerr << "Median denoising failed: " << err << "\n";
            break;
        case Denoiser::DenoiseMode::OIDNOnly:
            if (Denoiser::IsOIDNAvailable()) {
                ok = Denoiser::ApplyOIDN(renderer->GetRenderImage(), cameraRef.sRGB, denoisedOut, nullptr, &err);
                if (ok) std::cout << "Saved OIDN denoised image to " << denoisedOut << "\n";
                else std::cerr << "OIDN denoising failed: " << err << "\n";
            }
            else {
                std::cout << "OIDN denoiser not available in this build; skipping denoised output.\n";
            }
            break;
        case Denoiser::DenoiseMode::MedianThenOIDN:
            if (Denoiser::IsOIDNAvailable()) {
                ok = Denoiser::ApplyMedianThenOIDN(renderer->GetRenderImage(), cameraRef.sRGB, denoisedOut, nullptr, &err);
                if (ok) std::cout << "Saved median+OIDN denoised image to " << denoisedOut << "\n";
                else std::cerr << "Hybrid denoising failed: " << err << "\n";
            }
            else {
                std::cout << "OIDN denoiser not available; falling back to median filter only.\n";
                ok = Denoiser::ApplyMedian(renderer->GetRenderImage(), cameraRef.sRGB, denoisedOut, nullptr, &err);
                if (ok) std::cout << "Saved median-filtered image to " << denoisedOut << "\n";
                else std::cerr << "Median denoising failed: " << err << "\n";
            }
            break;
        default:
            break;
        }
    };

    auto renderAndSave = [&](const std::string& colorOut, const std::string& zOut, const std::string& samplesOut, const std::string& denoisedOut) {
        renderer->SetOutputPaths(colorOut, zOut, samplesOut, denoisedOut);
        renderer->GetRenderImage().Init(cameraRef.imgWidth, cameraRef.imgHeight);

        auto t0 = std::chrono::steady_clock::now();
        try {
            renderer->BeginRender();

            while (renderer->IsRendering()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            renderer->StopRender();
        }
        catch (const std::exception& e) {
            std::cerr << "Exception during rendering: " << e.what() << "\n";
            return std::chrono::duration<double, std::milli>(0);
        }
        catch (...) {
            std::cerr << "Unknown exception during rendering.\n";
            return std::chrono::duration<double, std::milli>(0);
        }
        auto t1 = std::chrono::steady_clock::now();

        renderer->GetRenderImage().ComputeZBufferImage();
        renderer->GetRenderImage().SaveImage(colorOut.c_str());
        renderer->GetRenderImage().SaveZImage(zOut.c_str());
        SaveSampleCountImageAndReport(renderer->GetRenderImage(), samplesOut);
        if (g_rtDenoiseMode != Denoiser::DenoiseMode::None) {
            applyDenoiser(denoisedOut);
        }

        return std::chrono::duration<double, std::milli>(t1 - t0);
    };

    if (renderSequence) {
        int pad = std::max(4, static_cast<int>(std::to_string(std::max(animFrames, 1)).size()));
        for (int frame = 0; frame < animFrames; ++frame) {
            double t = animStart + double(frame) / animFPS;
            if (animEnd > animStart && t > animEnd) t = animEnd;
            cameraRef.EvaluateAtTime(t);
            auto [colorOut, zOut, samplesOut, denoisedOut] = formatBaseNames(outPrefix, frame, pad);
            std::cout << "Rendering frame " << frame << " at t=" << t << "s\n";
            auto ms = renderAndSave(colorOut, zOut, samplesOut, denoisedOut);
            if (ms.count() > 0.0) {
                const double total_seconds = ms.count() / 1000.0;
                const int hours = static_cast<int>(total_seconds) / 3600;
                const int minutes = (static_cast<int>(total_seconds) % 3600) / 60;
                const int seconds = static_cast<int>(total_seconds) % 60;
                std::cout << "Frame " << frame << " render time: " << std::setfill('0') << std::setw(2) << hours << ":"
                    << std::setfill('0') << std::setw(2) << minutes << ":"
                    << std::setfill('0') << std::setw(2) << seconds << "\n";
            }
        }
        return 0;
    }

    if (singleTime >= 0.0) {
        cameraRef.EvaluateAtTime(singleTime);
        std::cout << "Single frame mode: Evaluating camera at time=" << singleTime << "s\n";
    }

    auto [colorOut, zOut, samplesOut, denoisedOut] = formatBaseNames(outPrefix, -1, 0);
    std::cout << "Starting render...\n";
    auto ms = renderAndSave(colorOut, zOut, samplesOut, denoisedOut);

    std::cout << (g_rtUseBBox ? "[bbox=on] " : "[bbox=off] ")
#if RT_USE_MESH_BVH
        << (g_rtUseMeshBVH ? "[bvh=on] " : "[bvh=off] ");
#else
        << "[bvh=compiled-out] ";
#endif

    if (ms.count() > 0.0) {
        const double total_seconds = ms.count() / 1000.0;
        const int hours = static_cast<int>(total_seconds) / 3600;
        const int minutes = (static_cast<int>(total_seconds) % 3600) / 60;
        const int seconds = static_cast<int>(total_seconds) % 60;
        std::cout << "Render time: " << std::setfill('0') << std::setw(2) << hours << ":"
            << std::setfill('0') << std::setw(2) << minutes << ":"
            << std::setfill('0') << std::setw(2) << seconds << "\n";
    }

    return 0;
}
