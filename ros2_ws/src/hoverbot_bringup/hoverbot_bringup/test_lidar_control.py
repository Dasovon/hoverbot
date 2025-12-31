#!/usr/bin/env python3
"""
Robot Activity Controller - Test Script

Publishes to /robot_active to control LiDAR motor state.

Usage:
  # Start motor (robot active)
  ros2 run hoverbot_bringup test_lidar_control --ros-args -p active:=true
  
  # Stop motor (robot idle)
  ros2 run hoverbot_bringup test_lidar_control --ros-args -p active:=false
  
  # Interactive mode
  ros2 run hoverbot_bringup test_lidar_control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys


class ActivityController(Node):
    """Simple controller for testing LiDAR power management."""
    
    def __init__(self):
        super().__init__('activity_controller')
        
        self.declare_parameter('active', True)
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        active = self.get_parameter('active').value
        rate = self.get_parameter('publish_rate').value
        
        self.publisher = self.create_publisher(Bool, '/robot_active', 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        
        self.msg = Bool()
        self.msg.data = active
        
        self.get_logger().info(f'Publishing robot_active: {active}')
        self.get_logger().info(f'Rate: {rate} Hz')
        self.get_logger().info('')
        self.get_logger().info('Press Ctrl+C to stop')
    
    def timer_callback(self):
        """Publish activity state."""
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = ActivityController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
