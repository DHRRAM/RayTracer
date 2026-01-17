#include "ShadingConfig.h"
#include "scene.h"
#include <algorithm>
#include <cmath>

// Slabs method for ray-box intersection.
// Returns true if the ray intersects the box for any parameter t in [0, t_max].
bool Box::IntersectRay(Ray const& r, float t_max) const {
    // Early out: empty box
    if (IsEmpty()) return false;

    float tmin = 0.0f;
    float tmax = t_max;

    // For each axis, compute intersection with slabs [pmin, pmax]
    for (int i = 0; i < 3; ++i) {
        const float orig = r.p[i];
        const float dir = r.dir[i];
        const float minv = pmin[i];
        const float maxv = pmax[i];

        // Handle rays parallel to the slabs explicitly to avoid INF/NaN
        if (fabs(dir) < 1e-20f) {
            // If the origin is outside the slab on this axis, no hit
            if (orig < minv || orig > maxv) return false;
            // Otherwise, this axis does not constrain t
            continue;
        }

        const float invD = 1.0f / dir;
        float t0 = (minv - orig) * invD;
        float t1 = (maxv - orig) * invD;
        if (t0 > t1) std::swap(t0, t1);

        // Clip to current interval
        if (t0 > tmin) tmin = t0;
        if (t1 < tmax) tmax = t1;

        // No overlap
        if (tmax < tmin) return false;
    }

    // There is an intersection interval. Accept if any part is within [0, t_max].
    // Note: allow origin inside the box (tmin < 0) as long as tmax >= 0.
    return tmax >= 0.0f && tmin <= t_max;
}
