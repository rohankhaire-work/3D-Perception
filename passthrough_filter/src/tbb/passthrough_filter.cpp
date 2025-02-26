#include "tbb/passthrough_filter.hpp"
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
  filtered_cloud->height = cloud->height;
  filtered_cloud->width = cloud->width;
  filtered_cloud->points.resize(cloud->points.size());

  // Params for cropbox filter
  Params params;
  params.max_x_range = 50.0f;
  params.min_x_range = -50.0f;
  params.max_y_range = 50.0f;
  params.min_y_range = -50.0f;
  params.max_z_range = 2.5f;
  params.min_z_range = -2.5f;

  // Iterate through the loaded point cloud using tbb
  tbb::parallel_for(size_t(0), cloud->points.size(), [&](size_t i) {
    // Store in a Point struct for convenience and readability
    iPoint point;
    point.x = cloud->points[i].x;
    point.y = cloud->points[i].y;
    point.z = cloud->points[i].z;

    // Apply the cropbox filter criteria
    if (point.x < params.min_x_range || point.x > params.max_x_range ||
        point.y < params.min_y_range || point.y > params.max_y_range ||
        point.z < params.min_z_range || point.z > params.max_z_range)
      return;

    // Assign filtered points to the nwe point cloud
    filtered_cloud->points[i] = cloud->points[i];
  });

  // Remove all points where x, y, and z are all zero
  filtered_cloud->points.erase(std::remove_if(filtered_cloud->points.begin(),
                                              filtered_cloud->points.end(),
                                              [](const pcl::PointXYZ &pt) {
                                                return pt.x == 0 && pt.y == 0 &&
                                                       pt.z == 0;
                                              }),
                               filtered_cloud->points.end());

  // Reassign the height and the width
  filtered_cloud->height = 1;
  filtered_cloud->width = filtered_cloud->points.size();

  // Save the filtered point cloud
  pcl::io::savePCDFileASCII("filtered_pcd.pcd", *filtered_cloud);
  spdlog::info("input point cloud size: {}", cloud->points.size());
  spdlog::info("fitlered point cloud size: {}", filtered_cloud->points.size());

  return 0;
}
