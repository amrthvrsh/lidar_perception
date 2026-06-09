#ifndef LIDAR_PERCEPTION__GROUND_REMOVAL_HPP_
#define LIDAR_PERCEPTION__GROUND_REMOVAL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace lidar_perception
{

class GroundRemovalNode : public rclcpp::Node
{
public:
  explicit GroundRemovalNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_;

  std::unique_ptr<pcl::SACSegmentation<pcl::PointXYZ>> seg_;
  std::unique_ptr<pcl::ExtractIndices<pcl::PointXYZ>> extract_;
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__GROUND_REMOVAL_HPP_
