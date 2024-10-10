#ifndef SAFETY_NODE__SAFETY_NODE_CPP_HPP_
#define SAFETY_NODE__SAFETY_NODE_CPP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <vector>

namespace safety_node
{

class SafetyNodeCpp : public rclcpp::Node
{
public:
  SafetyNodeCpp();
  
private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_brake();

  // Subscribers and Publisher
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

  // Parameters
  double threshold_itTc_;
  int debounce_count_;
  int debounce_threshold_;
  
  // Vehicle speed
  double speed_;

  // Throttling
  double last_safe_log_time_;
};

}  // namespace safety_node

#endif  // SAFETY_NODE__SAFETY_NODE_CPP_HPP_
