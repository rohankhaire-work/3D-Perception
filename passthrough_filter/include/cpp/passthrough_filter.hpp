#pragma once

#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct Params {
  float min_x_range;
  float max_x_range;
  float min_y_range;
  float max_y_range;
  float min_z_range;
  float max_z_range;
};

struct iPoint {
  float x;
  float y;
  float z;
};
