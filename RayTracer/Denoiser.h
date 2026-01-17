#pragma once

#include <string>
#include "renderer.h"

namespace Denoiser {

enum class DenoiseMode {
    None,
    MedianOnly,
    OIDNOnly,
    MedianThenOIDN
};

// Returns true if the binary was built with OIDN available.
bool IsOIDNAvailable();

// Applies a median filter on the linear color buffer (falls back to LDR if
// linear is unavailable). Kernel size is configured inside Denoiser.cpp.
// If outputPath is non-empty, writes an LDR PNG. If outputPixels is non-null,
// it writes the filtered LDR pixels into that buffer (must be width*height).
bool ApplyMedian(const RenderImage& image,
                 bool outputSRGB,
                 const std::string& outputPath,
                 Color24* outputPixels,
                 std::string* errMsg = nullptr);

// Runs Intel Open Image Denoise on the linear color buffer.
// If outputPath is non-empty, writes an LDR PNG. If outputPixels is non-null,
// it writes the denoised LDR pixels into that buffer (must be width*height).
// outputSRGB controls whether LDR is gamma-corrected to sRGB.
bool ApplyOIDN(const RenderImage& image,
               bool outputSRGB,
               const std::string& outputPath,
               Color24* outputPixels,
               std::string* errMsg = nullptr,
               const Color* overrideColor = nullptr);

// Applies a median prefilter and then runs OIDN using the filtered color as input.
// If OIDN is unavailable, returns false with an error message.
bool ApplyMedianThenOIDN(const RenderImage& image,
                         bool outputSRGB,
                         const std::string& outputPath,
                         Color24* outputPixels,
                         std::string* errMsg = nullptr);

} // namespace Denoiser
