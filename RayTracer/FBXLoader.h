//-------------------------------------------------------------------------------
///
/// \file       FBXLoader.h
/// \brief      Thin wrapper around the Autodesk FBX SDK for importing meshes
///             and cameras into the existing scene graph.
///
/// FBX support is optional; all entry points return false when the SDK is not
/// available or loading fails.
///
//-------------------------------------------------------------------------------

#pragma once

#include "scene.h"

class FBXLoader
{
public:
    // Returns true if the build has FBX support compiled in.
    static bool IsSupported();

    // Imports an FBX scene as children under the given parent node.
    static bool ImportScene(const char* filename,
                            Node& parent,
                            ObjFileList& objList,
                            MaterialList& materials,
                            TextureFileList& texFiles);

    // Loads the first (or named) FBX camera and populates animation keyframes.
    static bool LoadCamera(const char* filename,
                           const char* cameraName,
                           Camera& cameraOut);
};

