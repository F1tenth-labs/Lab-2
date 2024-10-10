#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.0

        # create ROS subscribers and publishers.
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10 
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        # Parameters
        self.threshold_iTTC = 1.0   # Time to collision threshold in seconds
        self.debounce_count = 0     # Current debounce count
        self.debounce_threshold = 3 # Number of consecutive detections before braking

        self.get_logger().info('Safety Node Python initialized.')

    def odom_callback(self, odom_msg):
        # update current speed
        self.speed = odom_msg.twist.twist.linear.x
        self.get_logger().debug(f'Current speed: {self.speed:.2f} m/s')

    def scan_callback(self, scan_msg):
        # Extract range measurements and angles
        ranges = np.array(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        num_ranges = len(ranges)

        # Compute angles for each range measurement
        angles = np.linspace(angle_min, angle_max, num_ranges)

        # Filter out invalid range measurements (NaN, inf, etc.)
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        if len(ranges) == 0:
            self.get_logger().warn('No valid range data received.')
            return

        # Calculate closing velocity (v_closing) = v * cos(theta)
        v_closing = self.speed * np.cos(angles)

        # Initialize iTTC with infinity
        ittc = np.full_like(ranges, np.inf)

        # Calculate iTTC where v_closing > 0
        positive_closing = v_closing > 0
        ittc[positive_closing] = ranges[positive_closing] / v_closing[positive_closing]

        # Handle any potential NaNs or Infs
        ittc = np.where(np.isfinite(ittc), ittc, np.inf)

        # Determine if any iTTC value is below the threshold
        collision_imminent = np.any(ittc < self.threshold_iTTC)

        if collision_imminent:
            self.debounce_count += 1
            self.get_logger().warn(f'Collision imminent! iTTC below threshold. Count: {self.debounce_count}')
            if self.debounce_count >= self.debounce_threshold:
                self.publish_brake()
                self.debounce_count = 0  # Reset debounce count after braking
        else:
            if self.debounce_count > 0:
                self.debounce_count -= 1  # Decrement debounce count if conditions are safe
            self.get_logger().info('Safe: No imminent collision detected.')


    def publish_brake(self):
        # Set speed to 0 and align the wheels to be straight
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
        brake_msg.drive.steering_angle = 0.0

        self.drive_pub.publish(brake_msg)
        self.get_logger().info('Emergency Brake has activated!')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        safety_node.get_logger().info('Safety Node shutting down.')
    finally:
        safety_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
