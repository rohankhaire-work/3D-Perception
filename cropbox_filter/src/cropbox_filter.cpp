#include "cropbox_filter.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ>("../data/sample.pcd", *cloud) == -1)
    {
      PCL_ERROR("Couldn't read the PCD file");
      return -1;
    }

  // Determine params of the input cloud - num points, size etc
  size_t height = cloud->height;
  size_t width = cloud->width;
  size_t num_points = height * width;

  // Total size of the points in Byte
  size_t input_data_size = num_points * POINT_XYZ_SIZE * SIZE_PER_CHANNEL;
  size_t filtered_data_size
    = num_points * CHANNELS_PER_POINT * SIZE_PER_CHANNEL;

  // Get the input point cloud
  // Store as pinned memory
  float *cloud_xyz = reinterpret_cast<float *>(cloud->points.data());

  // Copy to the device vector
  // CudaMalloc the size of input cloud
  float *input_pcd_ptr = nullptr;
  cudaMalloc((void **)&input_pcd_ptr, input_data_size);
  cudaMemcpy(input_pcd_ptr, cloud_xyz, input_data_size,
             cudaMemcpyHostToDevice);

  // Resulting filtered pointcloud for host memory
  thrust::host_vector<float> filtered_cloud(num_points * CHANNELS_PER_POINT);

  // Apply cropbox_filter kernel
  initiate_cropbox_kernel(input_pcd_ptr, filtered_cloud, num_points);

  // Convert the thrust result to pcd data
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcd(
    new pcl::PointCloud<pcl::PointXYZ>);

  for(size_t idx = 0; idx < filtered_cloud.size(); idx += CHANNELS_PER_POINT)
    {
      // garbage value is -99.0f
      if(filtered_cloud[idx] != -99.0f)
        {
          pcl::PointXYZ pt;
          pt.x = filtered_cloud[idx];
          pt.y = filtered_cloud[idx + 1];
          pt.z = filtered_cloud[idx + 2];

          // Create a point cloud from POINT XYZ
          filtered_pcd->emplace_back(pt);
        }
    }

  // Save the PCD
  pcl::io::savePCDFileASCII("filtered_pcd.pcd", *filtered_pcd);
  std::cout << "input point cloud size: " << num_points << "\n";
  std::cout << "filtered point cloud size: "
            << filtered_pcd->height * filtered_pcd->width << "\n";

  // Free the CUDA memory
  cudaFree(input_pcd_ptr);

  return 0;
}
