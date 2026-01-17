#include "Denoiser.h"
#include "lodepng.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <array>

#ifdef RT_ENABLE_OIDN
#include <OpenImageDenoise/oidn.hpp>
#endif

namespace Denoiser {

namespace {

constexpr int kMedianKernelSize = 3; // adjust here (3 or 5 are typical)
constexpr int kMedianSamples = kMedianKernelSize * kMedianKernelSize;

Color MedianAt(const Color* src, int width, int height, int x, int y, int radius) {
    std::array<float, kMedianSamples> rBuf{};
    std::array<float, kMedianSamples> gBuf{};
    std::array<float, kMedianSamples> bBuf{};
    int count = 0;
    for (int dy = -radius; dy <= radius; ++dy) {
        const int yy = std::clamp(y + dy, 0, height - 1);
        for (int dx = -radius; dx <= radius; ++dx) {
            const int xx = std::clamp(x + dx, 0, width - 1);
            const Color& c = src[yy * width + xx];
            rBuf[count] = c.r;
            gBuf[count] = c.g;
            bBuf[count] = c.b;
            ++count;
        }
    }
    const int mid = count / 2;
    auto median = [mid](std::array<float, kMedianSamples>& buf, int validCount) {
        std::nth_element(buf.begin(), buf.begin() + mid, buf.begin() + validCount);
        return buf[mid];
    };
    return Color(median(rBuf, count), median(gBuf, count), median(bBuf, count));
}

bool MedianFilterToLinear(const RenderImage& image,
                          std::vector<Color>& dst,
                          std::string* errMsg) {
    const int width = image.GetWidth();
    const int height = image.GetHeight();
    if (width <= 0 || height <= 0) {
        if (errMsg) *errMsg = "Invalid image dimensions for median filter.";
        return false;
    }

    const int radius = kMedianKernelSize / 2;
    const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
    dst.resize(pixelCount);

    const Color* linear = image.GetLinearPixels();
    const Color* src = linear;
    std::vector<Color> ldrAsLinear;
    if (!src) {
        const Color24* ldr = image.GetPixels();
        if (!ldr) {
            if (errMsg) *errMsg = "No pixel data available for median filter.";
            return false;
        }
        ldrAsLinear.resize(pixelCount);
        for (size_t i = 0; i < pixelCount; ++i) {
            ldrAsLinear[i] = ldr[i].ToColor();
        }
        src = ldrAsLinear.data();
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            dst[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] =
                MedianAt(src, width, height, x, y, radius);
        }
    }

    return true;
}

} // namespace

bool IsOIDNAvailable() {
#ifdef RT_ENABLE_OIDN
    return true;
#else
    return false;
#endif
}

bool ApplyMedian(const RenderImage& image,
                 bool outputSRGB,
                 const std::string& outputPath,
                 Color24* outputPixels,
                 std::string* errMsg) {
    std::vector<Color> filtered;
    if (!MedianFilterToLinear(image, filtered, errMsg)) {
        return false;
    }

    const int width = image.GetWidth();
    const int height = image.GetHeight();
    const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<Color24> out(pixelCount);
    for (size_t i = 0; i < pixelCount; ++i) {
        Color c = filtered[i];
        c.ClampMin(0.0f);
        c.ClampMax(1.0f);
        if (outputSRGB) {
            c = c.Linear2sRGB();
        }
        out[i] = Color24(
            static_cast<uint8_t>(std::clamp(c.r * 255.0f, 0.0f, 255.0f)),
            static_cast<uint8_t>(std::clamp(c.g * 255.0f, 0.0f, 255.0f)),
            static_cast<uint8_t>(std::clamp(c.b * 255.0f, 0.0f, 255.0f))
        );
    }

    if (outputPixels) {
        std::copy(out.begin(), out.end(), outputPixels);
    }
    if (!outputPath.empty()) {
        const unsigned encodeErr = lodepng::encode(outputPath.c_str(), reinterpret_cast<unsigned char*>(&out[0].r), width, height, LCT_RGB, 8);
        if (encodeErr != 0) {
            if (errMsg) *errMsg = lodepng_error_text(encodeErr);
            return false;
        }
    }

    return true;
}

bool ApplyOIDN(const RenderImage& image,
               bool outputSRGB,
               const std::string& outputPath,
               Color24* outputPixels,
               std::string* errMsg,
               const Color* overrideColor) {
#ifndef RT_ENABLE_OIDN
    if (errMsg) *errMsg = "OpenImageDenoise was not linked at build time.";
    return false;
#else
    try {
    const int width = image.GetWidth();
    const int height = image.GetHeight();
    if (width <= 0 || height <= 0) {
        if (errMsg) *errMsg = "Invalid image dimensions for denoising.";
        return false;
    }

    const Color* linear = image.GetLinearPixels();
    const Color24* ldr = image.GetPixels();
    const Color* albedo = image.GetAlbedo();
    const Vec3f* normal = image.GetNormal();

    const Color* colorSource = overrideColor ? overrideColor : linear;

    if (!colorSource && !ldr) {
        if (errMsg) *errMsg = "No pixel data available for denoising.";
        return false;
    }

    const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<float> color(pixelCount * 3);
    for (size_t i = 0; i < pixelCount; ++i) {
        Color c;
        if (colorSource) {
            c = colorSource[i];
        } else {
            c = Color(
                float(ldr[i].r) / 255.0f,
                float(ldr[i].g) / 255.0f,
                float(ldr[i].b) / 255.0f
            );
        }
        color[i * 3 + 0] = c.r;
        color[i * 3 + 1] = c.g;
        color[i * 3 + 2] = c.b;
    }

    std::vector<float> output(pixelCount * 3, 0.0f);

    // Try CPU first (reliable), then fall back to default if CPU plugin is unavailable.
    const std::vector<oidn::DeviceType> devicePrefs = {
        oidn::DeviceType::CPU,
        oidn::DeviceType::Default
    };
    oidn::DeviceRef device;
    std::string deviceInitErr = "No supported OIDN devices found.";
    for (auto dt : devicePrefs) {
        device = oidn::newDevice(dt);
        device.commit();
        const char* deviceError = nullptr;
        if (device.getError(deviceError) == oidn::Error::None) {
            deviceInitErr.clear();
            break;
        }
        deviceInitErr = deviceError ? deviceError : "Failed to initialize OIDN device.";
        device = oidn::DeviceRef(); // reset before trying next preference
    }
    if (!device) {
        if (errMsg) *errMsg = deviceInitErr;
        return false;
    }

    oidn::FilterRef filter = device.newFilter("RT");
    filter.setImage("color", color.data(), oidn::Format::Float3, width, height);
    filter.setImage("output", output.data(), oidn::Format::Float3, width, height);
    if (albedo) filter.setImage("albedo",
        const_cast<float*>(reinterpret_cast<const float*>(albedo)),
        oidn::Format::Float3, width, height);
    if (normal) filter.setImage("normal",
        const_cast<float*>(reinterpret_cast<const float*>(normal)),
        oidn::Format::Float3, width, height);
    if (albedo || normal) {
        filter.set("cleanAux", true);
    }
    filter.set("hdr", true);
    // Increase denoising strength for smoother results (default is 0.0, higher values = more smoothing)
    filter.set("strength", 1.0f);
    filter.commit();
    filter.execute();

    const char* oidnError = nullptr;
    const oidn::Error err = device.getError(oidnError);
    if (err != oidn::Error::None) {
        if (errMsg) *errMsg = oidnError ? oidnError : "Unknown OIDN error";
        return false;
    }

    std::vector<Color24> out8(pixelCount);
    for (size_t i = 0; i < pixelCount; ++i) {
        Color c(output[i * 3 + 0], output[i * 3 + 1], output[i * 3 + 2]);
        c.ClampMin(0.0f);
        c.ClampMax(1.0f);
        if (outputSRGB) {
            c = c.Linear2sRGB();
        }
        out8[i] = Color24(
            static_cast<uint8_t>(std::clamp(c.r * 255.0f, 0.0f, 255.0f)),
            static_cast<uint8_t>(std::clamp(c.g * 255.0f, 0.0f, 255.0f)),
            static_cast<uint8_t>(std::clamp(c.b * 255.0f, 0.0f, 255.0f))
        );
    }

    if (outputPixels) {
        std::copy(out8.begin(), out8.end(), outputPixels);
    }
    if (!outputPath.empty()) {
        const unsigned encodeErr = lodepng::encode(outputPath.c_str(), reinterpret_cast<unsigned char*>(&out8[0].r), width, height, LCT_RGB, 8);
        if (encodeErr != 0) {
            if (errMsg) *errMsg = lodepng_error_text(encodeErr);
            return false;
        }
    }

    return true;
    } catch (const std::exception& e) {
        if (errMsg) *errMsg = e.what();
        return false;
    } catch (...) {
        if (errMsg) *errMsg = "Unknown exception in OIDN denoiser.";
        return false;
    }
#endif
}

bool ApplyMedianThenOIDN(const RenderImage& image,
                         bool outputSRGB,
                         const std::string& outputPath,
                         Color24* outputPixels,
                         std::string* errMsg) {
    if (!IsOIDNAvailable()) {
        if (errMsg) *errMsg = "OpenImageDenoise was not linked at build time.";
        return false;
    }

    std::vector<Color> filtered;
    if (!MedianFilterToLinear(image, filtered, errMsg)) {
        return false;
    }

    return ApplyOIDN(image, outputSRGB, outputPath, outputPixels, errMsg, filtered.data());
}

} // namespace Denoiser
