#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

class Safety : public rclcpp::Node {
    // The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // Create ROS subscribers
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&Safety::drive_callback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        // Create ROS publisher
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        // Log initialization
        RCLCPP_INFO(this->get_logger(), "Safety Node C++ initialized.");
    }

private:
    double speed = 0.0;

    // Parameters
    double threshold_ttc_ = 1.0;
    int debounce_count_ = 0;
    int debounce_threshold_ = 3;

    // ROS Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // Update current speed
        speed = msg->twist.twist.linear.x;
        // Optional: Log current speed at a lower frequency
        RCLCPP_DEBUG(this->get_logger(), "Current speed: %.2f m/s", speed);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // Extract range measurements and angles
        const std::vector<float>& ranges = scan_msg->ranges;
        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;
        size_t num_ranges = ranges.size();

        // Compute angles for each range measurement
        std::vector<float> angles(num_ranges);
        for (size_t i = 0; i < num_ranges; ++i) {
            angles[i] = angle_min + i * angle_increment;
        }

        // Filter out invalid range measurements (NaN, inf, etc.)
        std::vector<float> valid_ranges;
        std::vector<float> valid_angles;
        for (size_t i = 0; i < num_ranges; ++i) {
            float range = ranges[i];
            if (std::isfinite(range)) {
                valid_ranges.push_back(range);
                valid_angles.push_back(angles[i]);
            }
        }

        if (valid_ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "No valid range data received.");
            return;
        }

        // Calculate closing velocity (v_closing) = speed * cos(theta)
        std::vector<float> v_closing(valid_ranges.size());
        for (size_t i = 0; i < valid_ranges.size(); ++i) {
            v_closing[i] = speed * std::cos(valid_angles[i]);
        }

        // Initialize iTTC with infinity
        std::vector<float> ittc(valid_ranges.size(), std::numeric_limits<float>::infinity());

        // Calculate iTTC where v_closing > 0
        for (size_t i = 0; i < valid_ranges.size(); ++i) {
            if (v_closing[i] > 0) {
                ittc[i] = valid_ranges[i] / v_closing[i];
            }
        }

        // Determine if any iTTC value is below the threshold
        bool collision_imminent = false;
        for (size_t i = 0; i < ittc.size(); ++i) {
            if (ittc[i] < threshold_ttc_) {
                collision_imminent = true;
                break;
            }
        }

        if (collision_imminent) {
            debounce_count_++;
            RCLCPP_WARN(this->get_logger(), "Collision imminent! iTTC below threshold. Count: %d", debounce_count_);
            if (debounce_count_ >= debounce_threshold_) {
                publish_brake();
                debounce_count_ = 0;  // Reset debounce count after braking
            }
        } else {
            if (debounce_count_ > 0) {
                debounce_count_--;  // Decrement debounce count if conditions are safe
            }
            RCLCPP_INFO(this->get_logger(), "Safe: No imminent collision detected.");
        }
    }

    void publish_brake()
    {
        // Publish a brake command to the /drive topic
        auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
        brake_msg.drive.speed = 0.0;  // Set speed to zero to initiate braking
        brake_msg.drive.steering_angle = 0.0;  // Maintain current steering angle

        drive_publisher_->publish(brake_msg);
        RCLCPP_INFO(this->get_logger(), "Emergency brake activated!");
    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
