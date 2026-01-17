#pragma once

#include "ShadingConfig.h"
#include "scene.h"
#include <vector>
#include <utility>

inline void CollectSceneNodeObjects(const Scene& scene, vector<pair<const Node*,
    const Object*>>& nodeObjects) {
    // Helper function
    auto collect = [&](const Node* node, auto& collect_ref) -> void {
        if (!node) return;
        if (node->GetNodeObj()) nodeObjects.emplace_back(node, node->GetNodeObj());
        for (int i = 0; i < node->GetNumChild(); ++i)
            collect_ref(node->GetChild(i), collect_ref);
    };
    collect(&scene.rootNode, collect);
}
