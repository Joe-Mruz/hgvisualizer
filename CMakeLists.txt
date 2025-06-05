cmake_minimum_required(VERSION 3.16)
project(HGVisualizerApp)

set(CMAKE_CXX_STANDARD 20)

# Path to your SDL3 installation
set(SDL3_DIR "D:/sdl/SDL3-devel-3.2.14-VC/SDL3-3.2.14")  # <-- Change this to your actual SDL3 path

# Path to your SDL3_ttf installation
set(SDL3_TTF_DIR "D:/sdl/SDL3_ttf-devel-3.1.0-VC/SDL3_ttf-3.1.0")  # <-- Change this to your actual SDL3_ttf path

# Set CUDA Toolkit root if not in default location
set(CUDAToolkit_ROOT "D:/nvidia cuda/v12.9")
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

add_executable(HGVisualizerApp
    src/main.cc src/config.cc src/ui.cc src/model.cc src/allocator.cc src/layout.cc src/layout_cuda.cu
)

target_include_directories(HGVisualizerApp PRIVATE
    ${SDL3_DIR}/include
    ${SDL3_TTF_DIR}/include
    ${CUDAToolkit_INCLUDE_DIRS}
)

link_directories(${SDL3_DIR}/lib/x64 ${SDL3_TTF_DIR}/lib/x64)
target_link_libraries(HGVisualizerApp PRIVATE
    CUDA::cudart
    "${SDL3_DIR}/lib/x64/SDL3.lib"
    "${SDL3_TTF_DIR}/lib/x64/SDL3_ttf.lib"
)

# Copy SDL3.dll to the output directory after building
# This ensures that the application can find the SDL3.dll at runtime.
add_custom_command(TARGET HGVisualizerApp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SDL3_DIR}/lib/x64/SDL3.dll"
        "$<TARGET_FILE_DIR:HGVisualizerApp>/SDL3.dll"
)

# Copy SDL3_ttf.dll to the output directory after building
add_custom_command(TARGET HGVisualizerApp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SDL3_TTF_DIR}/lib/x64/SDL3_ttf.dll"
        "$<TARGET_FILE_DIR:HGVisualizerApp>/SDL3_ttf.dll"
)

# Copy res directory to the output directory after building
add_custom_command(TARGET HGVisualizerApp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/res"
        "$<TARGET_FILE_DIR:HGVisualizerApp>/res"
)

# Copy CUDA runtime DLLs to the output directory after building
add_custom_command(TARGET HGVisualizerApp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CUDAToolkit_ROOT}/bin/cudart64_12.dll"
        "$<TARGET_FILE_DIR:HGVisualizerApp>/"
)

# Build project with:
# cmake --build build --config Debug
# or
# cmake --build build --config Release