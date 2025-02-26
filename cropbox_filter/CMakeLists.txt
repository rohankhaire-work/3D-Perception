cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(cropbox_filter CXX CUDA)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
                    
find_package(PCL 1.2 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(TBB REQUIRED)
find_package(spdlog REQUIRED)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -diag-suppress=20012")

add_executable(cropbox_filter_cuda src/cuda/cropbox_filter.cpp
                        src/cuda/cropbox_kernel.cu)

target_include_directories(cropbox_filter_cuda PRIVATE include ${PCL_INCLUDE_DIRS} ${CUDAToolkit_INCLUDE_DIR})
target_link_libraries(cropbox_filter_cuda PRIVATE ${PCL_LIBRARIES} ${CUDAToolkit_LIBRARY_DIR}/libcudart.so)

add_executable(cropbox_filter_tbb src/tbb/cropbox_filter.cpp)
target_include_directories(cropbox_filter_tbb PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(cropbox_filter_tbb PRIVATE ${PCL_LIBRARIES} spdlog::spdlog_header_only TBB::tbb)

