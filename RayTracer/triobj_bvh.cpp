#include "objects.h"
#include <cmath>
#include "BVHConfig.h"

// Toggle to compare with/without BVH without changing code elsewhere.
#ifndef RT_USE_MESH_BVH
#define RT_USE_MESH_BVH 1
#endif

#if RT_USE_MESH_BVH

// Recursive BVH traversal
bool TriObj::TraceBVHNode(Ray const& ray, HitInfo& hInfo, int hitSide, unsigned int nodeID) const {
    // Cull by node bounds
    const float* b = bvh.GetNodeBounds(nodeID);
    const Box nodeBox(b);
    if (!nodeBox.IntersectRay(ray, hInfo.z)) return false;

    bool hit = false;
    if (bvh.IsLeafNode(nodeID)) {
        const unsigned int count = bvh.GetNodeElementCount(nodeID);
        const unsigned int* elems = bvh.GetNodeElements(nodeID);
        for (unsigned int i = 0; i < count; ++i) {
            hit |= IntersectTriangle(ray, hInfo, hitSide, elems[i]);
        }
    } else {
        const unsigned int c0 = bvh.GetFirstChildNode(nodeID);
        const unsigned int c1 = bvh.GetSecondChildNode(nodeID);
        // Traverse children
        hit |= TraceBVHNode(ray, hInfo, hitSide, c0);
        hit |= TraceBVHNode(ray, hInfo, hitSide, c1);
    }
    return hit;
}

#endif