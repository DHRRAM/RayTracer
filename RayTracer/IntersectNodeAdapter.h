#pragma once

#include "ShadingConfig.h"
#include "scene.h"
#include "BBoxConfig.h"
#include <cmath>

#ifndef RT_USE_BBOX
#define RT_USE_BBOX 1
#endif

// Extended hit info that can track the light object
struct ExtendedHitInfo : public HitInfo {
    const Light* hitLight = nullptr;
    
    void Init() {
        HitInfo::Init();
        hitLight = nullptr;
    }
};

// Internal helper that assumes the input ray is already in the current node's local coordinates
inline bool IntersectNodeLocal(const Node* node, const Ray& localRay, HitInfo& outHit, int hitSide) {
    bool anyHit = false;
    const bool bboxEnabled = (RT_USE_BBOX != 0) && g_rtUseBBox;

    HitInfo bestLocal; bestLocal.Init();
    bestLocal.z = outHit.z; // carry current upper bound

    const Box& childUnionBox = node->GetChildBoundBox();
    const bool childUnionMiss = bboxEnabled &&
                                (!childUnionBox.IsEmpty() && !childUnionBox.IntersectRay(localRay, bestLocal.z));

    // Test object at this node
    if (const Object* obj = node->GetNodeObj()) {
        bool bboxOK = true;
        if (bboxEnabled) {
            const Box ob = obj->GetBoundBox();
            bboxOK = ob.IsEmpty() || ob.IntersectRay(localRay, bestLocal.z);
        }
        if (bboxOK) {
            HitInfo h; h.Init(); h.z = bestLocal.z;
            if (obj->IntersectRay(localRay, h, hitSide)) {
                if (h.z > 0.0f && h.z < bestLocal.z) {
                    bestLocal = h;
                    bestLocal.node = node;
                    anyHit = true;
                }
            }
        }
    }

    // Recurse to children
    if (!childUnionMiss) {
        const int childCount = node->GetNumChild();
        for (int c = 0; c < childCount; ++c) {
            const Node* child = node->GetChild(c);
            if (!child) continue;

            // Early-out culling per child
            bool childMaybeHit = true;
            Ray childRay = child->ToNodeCoords(localRay); // transform once for culling and traversal
            if (bboxEnabled) {
                childMaybeHit = false;
                const Box& childChildBox = child->GetChildBoundBox();
                if (!childChildBox.IsEmpty() && childChildBox.IntersectRay(childRay, bestLocal.z)) {
                    childMaybeHit = true;
                }
                if (const Object* childObj = child->GetNodeObj()) {
                    const Box ob = childObj->GetBoundBox();
                    if (ob.IsEmpty() || ob.IntersectRay(childRay, bestLocal.z)) {
                        childMaybeHit = true;
                    }
                }
            }

            if (!childMaybeHit) continue;

            HitInfo childLocal; childLocal.Init();
            childLocal.z = bestLocal.z;

            // Call local version since we already transformed the ray
            if (IntersectNodeLocal(child, childRay, childLocal, hitSide)) {
                if (childLocal.z > 0.0f && childLocal.z < bestLocal.z) {
                    bestLocal = childLocal;
                    anyHit = true;
                }
            }
        }
    }

    if (anyHit) {
        node->FromNodeCoords(bestLocal);
        outHit = bestLocal;
    }
    return anyHit;
}

// Public entry: transform to local, delegate to the local version
inline bool IntersectNode(const Node* node, const Ray& inRay, HitInfo& outHit, int hitSide = HIT_FRONT) {
    Ray localRay = node->ToNodeCoords(inRay);
    HitInfo localHit; localHit.Init();
    localHit.z = outHit.z; // upper bound from caller

    const bool hit = IntersectNodeLocal(node, localRay, localHit, hitSide);
    if (hit) outHit = localHit; // already transformed back by IntersectNodeLocal
    return hit;
}

// Scene-aware intersection that also tests renderable lights
inline bool IntersectSceneWithLights(const Scene* scene, const Ray& ray, ExtendedHitInfo& outHit, int hitSide = HIT_FRONT) {
    bool anyHit = false;
    ExtendedHitInfo bestHit;
    bestHit.Init();
    bestHit.z = outHit.z;

    // First, intersect with the scene tree
    HitInfo sceneHit;
    sceneHit.Init();
    sceneHit.z = bestHit.z;
    if (IntersectNode(&scene->rootNode, ray, sceneHit, hitSide)) {
        // Copy only the HitInfo portion explicitly to avoid aliasing surprises,
        // then set the extra light pointer.
        static_cast<HitInfo&>(bestHit) = sceneHit;
        bestHit.hitLight = nullptr;
        anyHit = true;
    }

    // Also test all renderable lights
    for (const Light* light : scene->lights) {
        if (light && light->IsRenderable()) {
            HitInfo lightHit;
            lightHit.Init();
            lightHit.z = bestHit.z;

            if (light->IntersectRay(ray, lightHit, hitSide)) {
                if (lightHit.z > 0.0f && lightHit.z < bestHit.z) {
                    static_cast<HitInfo&>(bestHit) = lightHit;
                    bestHit.hitLight = light;
                    bestHit.light = true;
                    anyHit = true;
                }
            }
        }
    }

    if (anyHit) {
        outHit = bestHit;
    }
    return anyHit;
}
