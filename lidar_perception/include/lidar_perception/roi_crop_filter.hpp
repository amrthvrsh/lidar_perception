#ifndef LIDAR_PERCEPTION__ROI_CROP_FILTER_HPP_
#define LIDAR_PERCEPTION__ROI_CROP_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_perception
{

class RoiCropFilterNode : public rclcpp::Node
{
public:
  explicit RoiCropFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // ROI bounds stored as plain floats — no PCL dependency in header
  float x_min_, x_max_;
  float y_min_, y_max_;
  float z_min_, z_max_;
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__ROI_CROP_FILTER_HPP_
