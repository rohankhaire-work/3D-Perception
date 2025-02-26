#include "cpp/passthrough_filter.hpp"
#include <spdlog/spdlog.h>

int main(int argc, char *argv[]) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/sample.pcd", *cloud) == -1) {
    PCL_ERROR("Couldn't read the PCD file");
    return -1;
  }

  // filtered cloud storage
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Params for cropbox filter
  Params params;
  params.max_x_range = 50.0f;
  params.min_x_range = -50.0f;
  params.max_y_range = 50.0f;
  params.min_y_range = -50.0f;
  params.max_z_range = 2.5f;
  params.min_z_range = -2.5f;

  // Iterate through the loaded point cloud using tbb
  for (size_t idx = 0; idx < cloud->points.size(); ++idx) {
    // Store in a Point struct for convenience and readability
    iPoint point;
    point.x = cloud->points[idx].x;
    point.y = cloud->points[idx].y;
    point.z = cloud->points[idx].z;

    // Apply the cropbox filter criteria
    if (point.x < params.min_x_range || point.x > params.max_x_range ||
        point.y < params.min_y_range || point.y > params.max_y_range ||
        point.z < params.min_z_range || point.z > params.max_z_range)
      continue;

    pcl::PointXYZ pcd_pt;
    pcd_pt.x = point.x;
    pcd_pt.y = point.y;
    pcd_pt.z = point.z;

    // Assign filtered points to the nwe point cloud
    filtered_cloud->emplace_back(pcd_pt);
  }

  // Save the filtered point cloud
  pcl::io::savePCDFileASCII("filtered_pcd.pcd", *filtered_cloud);
  spdlog::info("input point cloud size: {}", cloud->points.size());
  spdlog::info("fitlered point cloud size: {}", filtered_cloud->points.size());

  return 0;
}
