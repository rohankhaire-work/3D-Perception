#pragma once

#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <oneapi/tbb/parallel_for.h>

struct Params
{
  float min_x_range;
  float max_x_range;
  float min_y_range;
  float max_y_range;
  float min_z_range;
  float max_z_range;
};

struct iPoint
{
  float x;
  float y;
  float z;
};

constexpr int POINT_XYZ_SIZE{4};
constexpr int CHANNELS_PER_POINT{3};
constexpr int SIZE_PER_CHANNEL{4};
constexpr int POINT_XYZ_OFFSET = POINT_XYZ_SIZE * POINT_XYZ_SIZE;
