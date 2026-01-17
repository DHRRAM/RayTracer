<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/caf1f554-f979-4e77-8721-65cbcaf9b617" />
Warm version. Resolution: 1920 x 1080. Render Time: 3:04:49. Min samples: 128. Max samples: 1024. OIDN applied. Modeled in Houdini. Cereal is done with RBD.

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/b286abe1-866e-442f-8cee-22801aa7ef2c" />
Cool Version. Resolution: 1920 x 1080. Render Time: 02:45:11. Min samples: 128. Max samples: 1024. OIDN applied. Modeled in Houdini. Cereal is done with RBD.

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/2cc63de5-ba11-4d4f-8ef3-95481c54723c" />
Samples (Adaptive Sampling with Min: 128, Max: 1024)

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/83fdc5d4-f656-4091-85ca-ad3d76d576ea" />
Zbuffer

Ray Tracer README

Author:
Dhruv Ram

Date:
01/15/2026

Compilation Instructions:
1. Open Visual Studio 2022 and open the Project Folder.
2. Select the "CMakeLists.txt" file in \RayTracerNew for the CMake Integration Source Directory.
3. Select x64 Debug or x64 Release. (Release is recommended for faster renders.)
4. Select "Start Without Debugging".
5. This produces RayTracer.exe in the out\build\x64-debug\RayTracer or out\build\x64-release\RayTracer folder.
6. To render the scene provided, move the .xml, \objects folder, and \textures folder into \out\build\x64-release\RayTracer.

Running the Program:

After compiling, change directories to RayTracer.exe and run from the command line: RayTracer.exe scene.xml

- To check OpenGL view:
RayTracer.exe scene.xml --preview

- To render without bounding box:
RayTracer.exe scene.xml --nobbox

- To render without BVH:
RayTracer.exe scene.xml --nobvh

- To render with Tent Filter (default):
RayTracer.exe scene.xml --tent

- To render with Gaussian Filter:
RayTracer.exe scene.xml --gaussian

- To render with Mitchell-Netravali Filter:
RayTracer.exe scene.xml --mitchell

- To render with Lanczos Filter:
RayTracer.exe scene.xml --lanczos

Notes:
- Optional Intel Open Image Denoise integration is available (set RT_ENABLE_OIDN=ON and provide OpenImageDenoise via CMake). When enabled, a denoised output named render_denoised.png is written alongside render.png. Disable with RT_ENABLE_OIDN=OFF if OIDN is not installed.
- Integration of Autodesk FBX SDK is currently being implemented.
