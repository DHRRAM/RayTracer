#pragma once

#include "ShadingConfig.h"
#include "scene.h"

// Abstract base class for shading models
class ShadingModel {
public:
    virtual ~ShadingModel() = default;
    virtual Color Shade(const Ray& ray, const HitInfo& hInfo, const LightList& lights, int bounceCount) const = 0;
};

// Blinn-Phong shading implementation
class BlinnPhongShading : public ShadingModel {
public:
    Color Shade(const Ray& ray, const HitInfo& hInfo, const LightList& lights, int bounceCount) const override;
};
