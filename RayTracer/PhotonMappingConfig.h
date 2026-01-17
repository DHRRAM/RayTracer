#pragma once

// Global photon mapping configuration flags
// These control whether photon mapping is used for shading

namespace PhotonMappingConfig {
    // Enable photon tracing / caustics support even if indirect gathering is disabled
    constexpr bool USE_PHOTON_MAP_DIRECT = false;
    
    // Set to true to use photon mapping for indirect illumination  
    constexpr bool USE_PHOTON_MAP_INDIRECT = true;
    
    // Helper to check if photon mapping is enabled at all
    constexpr bool IsPhotonMappingEnabled() {
        return USE_PHOTON_MAP_DIRECT || USE_PHOTON_MAP_INDIRECT;
    }
}
