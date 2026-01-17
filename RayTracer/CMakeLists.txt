# CMakeList.txt : CMake project for RayTracer, include source and define
# project specific logic here.
#

set(SOURCES
    Main.cpp
    RayCaster.cpp
    xmlload.cpp
    lodepng.cpp
    viewport.cpp
    tinyxml2.cpp
    objects.cpp
    PhotonMapBuilder.cpp
    Shading.cpp
    Shading_Legacy.cpp
    scene_box.cpp
    triobj_bvh.cpp
    texture.cpp
    Denoiser.cpp
    FBXLoader.cpp
)

if(RT_ENABLE_FBX)
    list(APPEND SOURCES FBXLoader.cpp)
endif()

include_directories("freeglut/include")

# Add source to this project's executable.
add_executable (RayTracer ${SOURCES} "BVHConfig.h")

target_include_directories(RayTracer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++20")
endif()

# Optional FBX SDK integration
option(RT_ENABLE_FBX "Enable FBX scene loading" OFF)
set(FBXSDK_ROOT "" CACHE PATH "Path to Autodesk FBX SDK (root containing include/lib)")
if(RT_ENABLE_FBX)
    if(NOT FBXSDK_ROOT)
        set(_fbx_default_root "${CMAKE_SOURCE_DIR}/fbx_sdk")
        if(EXISTS "${_fbx_default_root}")
            set(FBXSDK_ROOT "${_fbx_default_root}")
        endif()
    endif()

    if(FBXSDK_ROOT AND EXISTS "${FBXSDK_ROOT}/include/fbxsdk.h")
        target_include_directories(RayTracer PRIVATE "${FBXSDK_ROOT}/include")

        set(_fbx_lib_dir "")
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            foreach(_cand
                "${FBXSDK_ROOT}/lib/vs2022/x64"
                "${FBXSDK_ROOT}/lib/vs2019/x64"
                "${FBXSDK_ROOT}/lib/x64")
                if(EXISTS "${_cand}")
                    set(_fbx_lib_dir "${_cand}")
                    break()
                endif()
            endforeach()
        else()
            foreach(_cand
                "${FBXSDK_ROOT}/lib/vs2022/x86"
                "${FBXSDK_ROOT}/lib/vs2019/x86"
                "${FBXSDK_ROOT}/lib/x86")
                if(EXISTS "${_cand}")
                    set(_fbx_lib_dir "${_cand}")
                    break()
                endif()
            endforeach()
        endif()

        if(_fbx_lib_dir)
            # Append build type subdirectory
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                set(_fbx_lib_dir "${_fbx_lib_dir}/debug")
            else()
                set(_fbx_lib_dir "${_fbx_lib_dir}/release")
            endif()
            # Choose library names based on build type
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                set(_fbx_lib_names libfbxsdk-md libfbxsdk fbxsdk)
                set(_xml_lib_names libxml2-md libxml2)
                set(_zlib_lib_names zlib-md zlib)
            else()
                set(_fbx_lib_names libfbxsdk libfbxsdk-md fbxsdk)
                set(_xml_lib_names libxml2 libxml2-md)
                set(_zlib_lib_names zlib zlib-md)
            endif()
            find_library(FBXSDK_LIBRARY
                NAMES ${_fbx_lib_names}
                PATHS "${_fbx_lib_dir}"
                NO_DEFAULT_PATH)
            if(FBXSDK_LIBRARY)
                # Find and link libxml2 and zlib dependencies
                find_library(LIBXML2_LIBRARY
                    NAMES ${_xml_lib_names}
                    PATHS "${_fbx_lib_dir}"
                    NO_DEFAULT_PATH)
                find_library(ZLIB_LIBRARY
                    NAMES ${_zlib_lib_names}
                    PATHS "${_fbx_lib_dir}"
                    NO_DEFAULT_PATH)
                target_link_libraries(RayTracer PRIVATE ${FBXSDK_LIBRARY})
                if(LIBXML2_LIBRARY)
                    target_link_libraries(RayTracer PRIVATE ${LIBXML2_LIBRARY})
                endif()
                if(ZLIB_LIBRARY)
                    target_link_libraries(RayTracer PRIVATE ${ZLIB_LIBRARY})
                endif()
                target_compile_definitions(RayTracer PRIVATE RT_ENABLE_FBX=1 FBXSDK_SHARED)
                if(WIN32)
                    set(_fbx_dll_candidates
                        "${_fbx_lib_dir}/libfbxsdk.dll"
                        "${_fbx_lib_dir}/fbxsdk.dll")
                    foreach(_dll ${_fbx_dll_candidates})
                        if(EXISTS "${_dll}")
                            add_custom_command(TARGET RayTracer POST_BUILD
                                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                                    "${_dll}"
                                    $<TARGET_FILE_DIR:RayTracer>)
                            break()
                        endif()
                    endforeach()
                endif()
            else()
                message(WARNING "FBX SDK library not found under ${_fbx_lib_dir}; FBX support disabled.")
            endif()
        else()
            message(WARNING "FBX SDK library directory not found under ${FBXSDK_ROOT}; FBX support disabled.")
        endif()
    else()
        message(STATUS "FBX SDK not found (set FBXSDK_ROOT to enable FBX import).")
    endif()
endif()

# Link the freeglut library to your executable
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    find_library(FREEGLUT_LIBRARY freeglut PATHS "${CMAKE_CURRENT_SOURCE_DIR}/freeglut/lib/x64" NO_DEFAULT_PATH)
else()
    find_library(FREEGLUT_LIBRARY freeglut PATHS "${CMAKE_CURRENT_SOURCE_DIR}/freeglut/lib/x86" NO_DEFAULT_PATH)
endif()
if(FREEGLUT_LIBRARY)
    target_link_libraries(RayTracer PRIVATE ${FREEGLUT_LIBRARY})
else()
    message(FATAL_ERROR "FreeGLUT library not found")
endif()

option(RT_USE_BBOX "Enable bounding box culling in IntersectNode" ON)

if(RT_USE_BBOX)
    target_compile_definitions(RayTracer PRIVATE RT_USE_BBOX=1)
else()
    target_compile_definitions(RayTracer PRIVATE RT_USE_BBOX=0)
endif()

# ---------------------------------------------------------------------------
# Intel Open Image Denoise (OIDN)
# ---------------------------------------------------------------------------
# We ship a prebuilt Windows x64 OIDN zip (oidn.zip). If the user hasn't
# provided OpenImageDenoise via the environment/CMake cache, auto-unpack the
# bundled archive into the build tree and point OpenImageDenoise_DIR at it so
# denoising is enabled by default.
set(_oidn_from_bundle OFF)
set(_oidn_bundle_root "")
if(WIN32 AND (NOT OpenImageDenoise_DIR OR OpenImageDenoise_DIR MATCHES "-NOTFOUND"))
    set(_oidn_zip "${CMAKE_SOURCE_DIR}/oidn.zip")
    if(EXISTS "${_oidn_zip}")
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(_oidn_extract_dir "${CMAKE_BINARY_DIR}/oidn-prebuilt")
            file(GLOB _oidn_config_dirs LIST_DIRECTORIES TRUE
                "${_oidn_extract_dir}/oidn*/lib/cmake/OpenImageDenoise-*")
            if(NOT _oidn_config_dirs)
                message(STATUS "Extracting bundled OpenImageDenoise archive: ${_oidn_zip}")
                file(MAKE_DIRECTORY "${_oidn_extract_dir}")
                file(ARCHIVE_EXTRACT INPUT "${_oidn_zip}" DESTINATION "${_oidn_extract_dir}")
                file(GLOB _oidn_config_dirs LIST_DIRECTORIES TRUE
                    "${_oidn_extract_dir}/oidn*/lib/cmake/OpenImageDenoise-*")
            endif()
            if(_oidn_config_dirs)
                list(GET _oidn_config_dirs 0 _oidn_config_dir)
                # _oidn_config_dir = <root>/lib/cmake/OpenImageDenoise-2.3.0
                # We need the package root (<root>) to pick up the /bin DLLs.
                get_filename_component(_oidn_bundle_root "${_oidn_config_dir}/../../.." REALPATH)
                set(OpenImageDenoise_DIR "${_oidn_config_dir}" CACHE PATH "Path to bundled OpenImageDenoise package" FORCE)
                set(_oidn_from_bundle ON)
                message(STATUS "Using bundled OpenImageDenoise at ${OpenImageDenoise_DIR}")
            else()
                message(WARNING "Bundled OpenImageDenoise archive found, but no CMake config directory was detected after extraction.")
            endif()
        else()
            message(STATUS "Bundled OpenImageDenoise archive is 64-bit only; skipping auto-setup for this build.")
        endif()
    endif()
endif()

# Always try to find and link OpenImageDenoise if available
find_package(OpenImageDenoise QUIET CONFIG)
set(_oidn_extra_runtime_dlls "")
set(_oidn_pkg_root "")
if(OpenImageDenoise_FOUND)
    message(STATUS "OpenImageDenoise found: ${OpenImageDenoise_DIR}")
    set(_oidn_target "")
    if(TARGET OpenImageDenoise::OpenImageDenoise)
        set(_oidn_target OpenImageDenoise::OpenImageDenoise)
    elseif(TARGET OpenImageDenoise)
        set(_oidn_target OpenImageDenoise)
    endif()
    if(_oidn_target)
        target_link_libraries(RayTracer PRIVATE ${_oidn_target})
        target_compile_definitions(RayTracer PRIVATE RT_ENABLE_OIDN=1)
        message(STATUS "OpenImageDenoise linked successfully")
        # Derive the package root from the config path and copy all bin DLLs
        # so device plugins (CPU/GPU) and TBB are available at runtime.
        if(OpenImageDenoise_DIR)
            get_filename_component(_oidn_pkg_root "${OpenImageDenoise_DIR}/../../.." REALPATH)
            if(EXISTS "${_oidn_pkg_root}/bin")
                file(GLOB _oidn_extra_runtime_dlls "${_oidn_pkg_root}/bin/*.dll")
            endif()
        endif()
    else()
        message(WARNING "OpenImageDenoise package found but no import target detected; denoiser will be disabled.")
    endif()
else()
    message(STATUS "OpenImageDenoise not found; denoising will not be available at runtime.")
endif()

# Copy OIDN (and its dependent) DLLs next to the executable on Windows so runtime load works.
if(WIN32 AND OpenImageDenoise_FOUND)
    set(_oidn_copy_commands "")
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.21")
        list(APPEND _oidn_copy_commands
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:RayTracer>
                $<TARGET_FILE_DIR:RayTracer>)
    elseif(TARGET OpenImageDenoise::OpenImageDenoise)
        list(APPEND _oidn_copy_commands
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:OpenImageDenoise::OpenImageDenoise>
                $<TARGET_FILE_DIR:RayTracer>)
    endif()

    if(_oidn_extra_runtime_dlls)
        list(APPEND _oidn_copy_commands
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${_oidn_extra_runtime_dlls}
                $<TARGET_FILE_DIR:RayTracer>)
    endif()

    if(_oidn_copy_commands)
        add_custom_command(TARGET RayTracer POST_BUILD
            ${_oidn_copy_commands}
            COMMAND_EXPAND_LISTS)
    endif()
endif()

# Ensure the shading toggle macros (SHADING_USE_LEGACY / LEGACY_SHADING_API)
# are consistent across all translation units by forcing ShadingConfig.h
# to be included first.
target_compile_options(RayTracer PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/FIShadingConfig.h>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-include;ShadingConfig.h>
    $<$<CXX_COMPILER_ID:MSVC>:/EHsc>
)
