#pragma once

// Compile-time capability toggle (default true).
#ifndef RT_USE_MESH_BVH
#define RT_USE_MESH_BVH 1
#endif

// Runtime toggle for mesh BVH traversal
extern bool g_rtUseMeshBVH;