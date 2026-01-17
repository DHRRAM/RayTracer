<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/8eb49855-3794-42b5-85d9-1210d968205f" />

Warm version. Resolution: 1920 x 1080. Render Time: 3:04:49. Min samples: 128. Max samples: 1024. OIDN applied. Modeled in Houdini. Cereal is done with RBD.

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/7a92fb82-1f59-42b6-b172-53a52414db88" />

Cool Version. Resolution: 1920 x 1080. Render Time: 02:45:11. Min samples: 128. Max samples: 1024. OIDN applied. Modeled in Houdini. Cereal is done with RBD.

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/69f2dbac-6e48-4399-8db8-62cdb5a17fe6" />

Samples (Adaptive Sampling with Min: 128, Max: 1024)

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/79eaf7a4-e571-439e-ba28-3573543ad136" />

Zbuffer

# RayTracer

A **physically based ray tracing renderer** implemented in **C++**, developed as part of graduate-level computer graphics coursework.
This project focuses on **correct light transport**, **material realism**, and **extensible renderer architecture**, with an emphasis on clarity, correctness, and performance-conscious design.

---

## Features

### Core Ray Tracing

* Primary ray generation with **closest-hit intersection**
* Recursive ray evaluation for:

  * Perfect reflections
  * Refractions with Snell’s Law
  * Fresnel blending
* Shadow rays with support for **soft shadows**
* Configurable recursion depth

### Illumination & Shading

* **Phong / Blinn-Phong shading**
* Monte Carlo sampling for indirect illumination
* Physically motivated **Fresnel reflectance**
* Energy-conserving BRDF handling

### Global Illumination

* **Monte Carlo Path Tracing**
* **Photon Mapping**

  * Photon emission
  * Spatial photon storage
  * Radiance estimation
* Optional **Open Image Denoise (OIDN)** integration

### Sampling & Quality

* Stratified and low-discrepancy sampling
* Anti-aliasing via multi-sample pixel evaluation
* Importance sampling support (where applicable)

---

## Build & Run

### Requirements

* **C++17**
* **CMake 3.16+**
* OpenGL (for preview / visualization where applicable)
* Intel **Open Image Denoise** (optional)

### Compilation Instructions:
1. Open Visual Studio 2022 and open the Project Folder.
2. Select the "CMakeLists.txt" file in \RayTracerNew for the CMake Integration Source Directory.
3. Select x64 Debug or x64 Release. (Release is recommended for faster renders.)
4. Select "Start Without Debugging".
5. This produces RayTracer.exe in the out\build\x64-debug\RayTracer or out\build\x64-release\RayTracer folder.
6. To render the scene provided, move the .xml, \objects folder, and \textures folder into \out\build\x64-release\RayTracer.

### Running the Program:

After compiling, change directories to RayTracer.exe and run from the command line: RayTracer.exe scene.xml

* To check OpenGL view:
RayTracer.exe scene.xml --preview

* To render without bounding box:
RayTracer.exe scene.xml --nobbox

* To render without BVH:
RayTracer.exe scene.xml --nobvh

* To render with Tent Filter (default):
RayTracer.exe scene.xml --tent

* To render with Gaussian Filter:
RayTracer.exe scene.xml --gaussian

* To render with Mitchell-Netravali Filter:
RayTracer.exe scene.xml --mitchell

* To render with Lanczos Filter:
RayTracer.exe scene.xml --lanczos

---

## Output

* High-quality offline renders
* Configurable image resolution
* Progressive refinement (path tracing modes)
* PNG / HDR output support (depending on build config)

## Author

**Dhruv Ram**
MS Computing — Graphics & Visualization
University of Utah

Background in Animation & Visual Effects
Aspiring Tools Developer (Film & Games)

---

## License

This project is for **educational and portfolio purposes**.
Reuse with attribution where appropriate.

---
