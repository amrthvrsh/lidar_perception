#ifndef LIDAR_PERCEPTION__VOXEL_GRID_FILTER_HPP_
#define LIDAR_PERCEPTION__VOXEL_GRID_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>

namespace lidar_perception
{

class VoxelGridFilterNode : public rclcpp::Node
{
public:
  explicit VoxelGridFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  float leaf_size_;  // stored directly; filtering uses pcl::PCLPointCloud2 (type-agnostic)
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__VOXEL_GRID_FILTER_HPP_
