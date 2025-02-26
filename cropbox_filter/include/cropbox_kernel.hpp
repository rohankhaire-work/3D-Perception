#pragma once

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <cuda/std/span>

struct Params
{
  float min_x_range;
  float max_x_range;
  float min_y_range;
  float max_y_range;
  float min_z_range;
  float max_z_range;
};
