#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

namespace lidar_perception
{

class PipelineMonitorNode : public rclcpp::Node
{
public:
  PipelineMonitorNode(const rclcpp::NodeOptions & options)
  : Node("pipeline_monitor_node", options)
  {
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(5);

    std::string topic_orig = this->declare_parameter("topic_original", "/sensing/lidar/concatenated/pointcloud");
    std::string topic_roi = this->declare_parameter("topic_roi", "/lidar/points_roi");
    std::string topic_voxel = this->declare_parameter("topic_voxel", "/lidar/points_downsampled");
    std::string topic_ground_rem = this->declare_parameter("topic_obstacle", "/lidar/points_obstacle");
    std::string topic_cluster = this->declare_parameter("topic_cluster", "/lidar/clusters");

    sub_orig_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_orig, qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pts_orig_ = msg->width * msg->height; });
    sub_roi_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_roi, qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pts_roi_ = msg->width * msg->height; });
    sub_voxel_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_voxel, qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pts_voxel_ = msg->width * msg->height; });
    sub_ground_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_ground_rem, qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pts_ground_rem_ = msg->width * msg->height; });
    sub_cluster_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_cluster, qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pts_cluster_ = msg->width * msg->height; });

    timer_ = this->create_wall_timer(5s, std::bind(&PipelineMonitorNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Pipeline Monitor initialized. Will report stats every 5 seconds.");
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(),
      "\n--- LiDAR Perception Pipeline Status ---\n"
      "1. Original Point Cloud     : %d points\n"
      "2. After ROI Filtering      : %d points\n"
      "3. After Voxel Downsampling : %d points\n"
      "4. After Ground Removal     : %d points (obstacle)\n"
      "5. After Clustering         : %d points (in clusters)\n"
      "----------------------------------------",
      pts_orig_.load(), pts_roi_.load(), pts_voxel_.load(), pts_ground_rem_.load(), pts_cluster_.load());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_orig_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_roi_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_voxel_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ground_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cluster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic<int> pts_orig_{0};
  std::atomic<int> pts_roi_{0};
  std::atomic<int> pts_voxel_{0};
  std::atomic<int> pts_ground_rem_{0};
  std::atomic<int> pts_cluster_{0};
};

}  // namespace lidar_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_perception::PipelineMonitorNode)
