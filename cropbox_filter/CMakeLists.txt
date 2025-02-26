cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(cropbox_filter CXX CUDA)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
                    
find_package(PCL 1.2 REQUIRED)
find_package(CUDAToolkit REQUIRED)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -diag-suppress=20012")

add_executable(cropbox_filter src/cropbox_filter.cpp
                        src/cropbox_kernel.cu)

target_include_directories(cropbox_filter PRIVATE include ${PCL_INCLUDE_DIRS} ${CUDAToolkit_INCLUDE_DIR})
target_link_libraries(cropbox_filter PRIVATE ${PCL_LIBRARIES} ${CUDAToolkit_LIBRARY_DIR}/libcudart.so)
