cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(passthrough_filter CXX CUDA)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
                    
find_package(PCL 1.2 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(TBB REQUIRED)
find_package(spdlog REQUIRED)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -diag-suppress=20012")

add_executable(passthrough_filter_tbb src/tbb/passthrough_filter.cpp)
target_include_directories(passthrough_filter_tbb PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(passthrough_filter_tbb PRIVATE ${PCL_LIBRARIES} spdlog::spdlog_header_only TBB::tbb)

add_executable(passthrough_filter_cpp src/cpp/passthrough_filter.cpp)
target_include_directories(passthrough_filter_cpp PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(passthrough_filter_cpp PRIVATE ${PCL_LIBRARIES} spdlog::spdlog_header_only)


