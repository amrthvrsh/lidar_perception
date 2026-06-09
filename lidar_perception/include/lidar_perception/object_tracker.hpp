#ifndef LIDAR_PERCEPTION__OBJECT_TRACKER_HPP_
#define LIDAR_PERCEPTION__OBJECT_TRACKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "lidar_perception/msg/bounding_box3_d_array.hpp"
#include "lidar_perception/kalman_tracker.hpp"
#include <map>

namespace lidar_perception
{

class ObjectTrackerNode : public rclcpp::Node
{
public:
  explicit ObjectTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<lidar_perception::msg::BoundingBox3DArray>::SharedPtr pub_objects_;

  std::map<int, KalmanTracker> trackers_;
  int next_id_;
  rclcpp::Time last_time_;
  bool first_frame_;
  
  double max_association_dist_;
  int max_invisible_count_;
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__OBJECT_TRACKER_HPP_
