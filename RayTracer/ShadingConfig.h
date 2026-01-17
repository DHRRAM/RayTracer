#pragma once

// Global shading selection.
// Set SHADING_USE_LEGACY to 1 to enable the legacy shading path
// (Shading_Legacy.cpp + ShadeInfo API). Set to 0 to use the
// path-tracing integrator in Shading.cpp.
#ifndef SHADING_USE_LEGACY
#   ifdef LEGACY_SHADING_API
#       define SHADING_USE_LEGACY 1
#   else
#       define SHADING_USE_LEGACY 0
#   endif
#endif

// Keep the old API available when legacy shading is active.
#if SHADING_USE_LEGACY
#   ifndef LEGACY_SHADING_API
#       define LEGACY_SHADING_API
#   endif
#else
#   ifdef LEGACY_SHADING_API
#       undef LEGACY_SHADING_API
#   endif
#endif

namespace ShadingConfig {
    constexpr bool USE_LEGACY_SHADING = (SHADING_USE_LEGACY != 0);
    constexpr bool USE_PATH_TRACING   = !USE_LEGACY_SHADING;
}
