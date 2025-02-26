#include "cuda/cropbox_filter.hpp"
#include "cuda/cropbox_kernel.hpp"

__global__ void cropbox_filter_kernel(cuda::std::span<float> input_points,
                                      cuda::std::span<float> filtered_points,
                                      size_t num_points, const Params params)
{
  int const index = threadIdx.x + blockIdx.x * blockDim.x;
  if(index >= num_points)
    return;
  // Get the point from index
  const int pcd_idx = index * CHANNELS_PER_POINT;

  auto point = (reinterpret_cast<float4 *>(input_points.data()))[index];

  // Apply cropbox criteria
  if(point.x > params.min_x_range && point.x < params.max_x_range
     && point.y > params.min_y_range && point.y < params.max_y_range
     && point.z > params.min_z_range && point.z < params.max_z_range)
    return;

  // Store the point in a new array
  filtered_points[pcd_idx] = point.x;
  filtered_points[pcd_idx + 1] = point.y;
  filtered_points[pcd_idx + 2] = point.z;
}

void initiate_cropbox_kernel(float *input_cloud,
                             thrust::host_vector<float> &filtered_pcd,
                             size_t num_points)
{
  // Set cropbox params
  Params param;
  param.max_x_range = 8.0f;
  param.min_x_range = -8.0f;
  param.max_y_range = 8.0f;
  param.min_y_range = -8.0f;
  param.max_z_range = 0.5f;
  param.min_z_range = -2.5f;

  // Initalize result point cloud on the device
  thrust::device_vector<float> filtered_cloud(num_points * CHANNELS_PER_POINT);
  thrust::fill(filtered_cloud.begin(), filtered_cloud.end(), -99.0f);

  // Set the thread and blocks
  const int block_size{512};
  const int num_blocks = (num_points + block_size - 1) / block_size;

  // Call the kernel
  cropbox_filter_kernel<<<num_blocks, block_size>>>(
    cuda::std::span<float>(input_cloud, num_points * POINT_XYZ_SIZE),
    cuda::std::span<float>(thrust::raw_pointer_cast(filtered_cloud.data()),
                           filtered_cloud.size()),
    num_points, param);
  // Make sure kernel operations are done
  cudaDeviceSynchronize();

  // Copy to the host vector
  thrust::copy(filtered_cloud.begin(), filtered_cloud.end(),
               filtered_pcd.begin());
}
