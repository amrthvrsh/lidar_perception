#include "lidar_perception/voxel_grid_filter.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

namespace lidar_perception
{

VoxelGridFilterNode::VoxelGridFilterNode(const rclcpp::NodeOptions & options)
: Node("voxel_grid_filter_node", options)
{
  double leaf_size = this->declare_parameter("leaf_size", 0.1);
  leaf_size_ = static_cast<float>(leaf_size);

  std::string input_topic = this->declare_parameter("input_topic", "/lidar/points_roi");
  std::string output_topic = this->declare_parameter("output_topic", "/lidar/points_downsampled");

  auto sensor_qos = rclcpp::SensorDataQoS();
  sensor_qos.keep_last(5);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, sensor_qos,
    std::bind(&VoxelGridFilterNode::pointcloud_callback, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic, sensor_qos);

  RCLCPP_INFO(this->get_logger(), "Voxel Grid Filter initialized.\n  Input: %s\n  Output: %s\n  Leaf size: %.2f m",
    input_topic.c_str(), output_topic.c_str(), leaf_size);
}

void VoxelGridFilterNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto start_time = std::chrono::steady_clock::now();

  if (msg->data.empty()) {
    return;
  }

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);

  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
  
  // Operate directly on PCLPointCloud2 — type-agnostic, works with any LiDAR driver
  pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
  grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  grid.setInputCloud(std::make_shared<pcl::PCLPointCloud2>(pcl_pc2));
  grid.filter(*cloud_filtered);

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl_conversions::moveFromPCL(*cloud_filtered, output_msg);
  output_msg.header = msg->header;

  pub_->publish(output_msg);

  auto end_time = std::chrono::steady_clock::now();
  double dt_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  RCLCPP_DEBUG(this->get_logger(), "Voxel Downsample: %d -> %d pts [%.2f ms]",
    msg->width * msg->height, output_msg.width * output_msg.height, dt_ms);
}

}  // namespace lidar_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_perception::VoxelGridFilterNode)
