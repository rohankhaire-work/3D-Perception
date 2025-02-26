#pragma once

#include <cuda_runtime.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/host_vector.h>

constexpr int POINT_XYZ_SIZE{4};
constexpr int CHANNELS_PER_POINT{3};
constexpr int SIZE_PER_CHANNEL{4};
constexpr int POINT_XYZ_OFFSET=POINT_XYZ_SIZE*POINT_XYZ_SIZE;

void initiate_cropbox_kernel(float*, thrust::host_vector<float>&, size_t);

