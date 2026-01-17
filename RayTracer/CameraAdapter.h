#pragma once

#include "ShadingConfig.h"
#include "scene.h"
#include <cmath>

inline Ray GenerateCameraRay(const Camera& cam, int x, int y) {
    float fovRad = cam.fov * 3.14159265f / 180.0f;
    float halfY = tan(0.5f * fovRad);
    float aspect = (cam.imgHeight > 0) ? float(cam.imgWidth) / float(cam.imgHeight) : 1.0f;
    float halfX = halfY * aspect;

    float ndcX = ((x + 0.5f) / float(cam.imgWidth)) * 2.0f - 1.0f;
    float ndcY = 1.0f - ((y + 0.5f) / float(cam.imgHeight)) * 2.0f;

    float sx = ndcX * halfX;
    float sy = ndcY * halfY;

    Vec3f f = cam.dir;
    Vec3f r = (f.Cross(cam.up)).GetNormalized();
    Vec3f u = (r.Cross(f)).GetNormalized();

    Vec3f dir = r * sx + u * sy + f;
    dir.Normalize();

    return Ray(cam.pos, dir);
}
