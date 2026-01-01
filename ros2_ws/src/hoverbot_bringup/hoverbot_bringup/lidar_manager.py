#!/usr/bin/env python3
"""
LiDAR Manager Node

Intelligently controls RPLidar motor based on robot activity to save power.

Subscribes to:
  /robot_active (std_msgs/Bool) - True = start motor, False = stop motor

Service Clients:
  /start_motor (std_srvs/Empty) - Start RPLidar motor
  /stop_motor (std_srvs/Empty) - Stop RPLidar motor

Behavior:
  - On startup: Stop motor (default safe state)
  - On /robot_active True: Start motor if not already running
  - On /robot_active False: Stop motor if running
  - Debounce: Avoid redundant service calls
  - Timeout: Stop motor if no activity signal for 30 seconds
  - Bench test mode: Disable auto-stop for testing (motor stays on)

Usage:
  ros2 run hoverbot_bringup lidar_manager
  
  With bench test mode:
  ros2 run hoverbot_bringup lidar_manager --ros-args -p bench_test_mode:=true
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool
from std_srvs.srv import Empty


class LidarManager(Node):
    """
    Manages RPLidar motor state based on robot activity.
    
    Saves battery by stopping motor when robot is idle.
    """
    
    def __init__(self):
        super().__init__('lidar_manager')
        
        # Declare parameters
        self.declare_parameter('activity_timeout', 30.0)  # seconds
        self.declare_parameter('startup_delay', 2.0)  # seconds
        self.declare_parameter('bench_test_mode', False)  # disable auto-stop
        
        # Get parameters
        self.activity_timeout = self.get_parameter('activity_timeout').value
        self.bench_test_mode = self.get_parameter('bench_test_mode').value
        startup_delay = self.get_parameter('startup_delay').value
        
        # State tracking
        self.motor_running = True  # RPLidar starts with motor on by default
        self.last_activity_time = self.get_clock().now()
        self.last_service_call_time = None
        self.service_call_in_progress = False
        
        # Service clients
        self.start_motor_client = self.create_client(Empty, '/start_motor')
        self.stop_motor_client = self.create_client(Empty, '/stop_motor')
        
        # Subscriber
        self.activity_sub = self.create_subscription(
            Bool,
            '/robot_active',
            self.activity_callback,
            10
        )
        
        # Timer for watchdog (check for activity timeout)
        self.watchdog_timer = self.create_timer(5.0, self.watchdog_callback)
        
        self.get_logger().info('LiDAR Manager initialized')
        self.get_logger().info(f'Activity timeout: {self.activity_timeout}s')
        
        if self.bench_test_mode:
            self.get_logger().warn('⚠️  BENCH TEST MODE: Auto-stop disabled! Motor will stay on.')
        
        # Wait for services to be available
        self.get_logger().info('Waiting for RPLidar services...')
        
        # Create timer to stop motor after startup delay
        self.startup_timer = self.create_timer(
            startup_delay,
            self.startup_stop_motor
        )
    
    def startup_stop_motor(self):
        """
        Stop motor on startup (after delay) to save power.
        RPLidar starts with motor running by default.
        """
        self.get_logger().info('Startup: Stopping motor to save power...')
        self.stop_motor()
        self.startup_timer.cancel()
    
    def activity_callback(self, msg: Bool):
        """
        Handle robot activity state changes.
        
        In bench test mode, only responds to start (true), ignores stop (false).
        
        Args:
            msg: Bool message, True = active, False = idle
        """
        self.last_activity_time = self.get_clock().now()
        
        if msg.data:
            # Robot active - ensure motor is running
            if not self.motor_running:
                self.get_logger().info('Robot active - Starting motor')
                self.start_motor()
        else:
            # Robot idle - stop motor to save power
            # SKIP in bench test mode - motor stays on
            if self.bench_test_mode:
                return  # Don't stop in bench test mode
            
            if self.motor_running:
                self.get_logger().info('Robot idle - Stopping motor')
                self.stop_motor()
    
    def watchdog_callback(self):
        """
        Watchdog timer - stop motor if no activity for timeout period.
        
        Disabled in bench test mode.
        """
        # Skip watchdog in bench test mode
        if self.bench_test_mode:
            return
        
        if not self.motor_running:
            return  # Already stopped
        
        time_since_activity = (
            self.get_clock().now() - self.last_activity_time
        ).nanoseconds / 1e9
        
        if time_since_activity > self.activity_timeout:
            self.get_logger().warn(
                f'No activity for {time_since_activity:.1f}s - Stopping motor'
            )
            self.stop_motor()
    
    def start_motor(self):
        """
        Start RPLidar motor (asynchronously).
        """
        if self.service_call_in_progress:
            self.get_logger().debug('Service call already in progress, skipping')
            return
        
        if not self.start_motor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('start_motor service not available')
            return
        
        self.service_call_in_progress = True
        request = Empty.Request()
        
        future = self.start_motor_client.call_async(request)
        future.add_done_callback(self.start_motor_callback)
    
    def start_motor_callback(self, future):
        """
        Handle start_motor service response.
        """
        self.service_call_in_progress = False
        
        try:
            response = future.result()
            self.motor_running = True
            self.last_service_call_time = self.get_clock().now()
            self.get_logger().info('Motor started successfully')
        except Exception as e:
            self.get_logger().error(f'start_motor service call failed: {e}')
    
    def stop_motor(self):
        """
        Stop RPLidar motor (asynchronously).
        """
        if self.service_call_in_progress:
            self.get_logger().debug('Service call already in progress, skipping')
            return
        
        if not self.stop_motor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('stop_motor service not available')
            return
        
        self.service_call_in_progress = True
        request = Empty.Request()
        
        future = self.stop_motor_client.call_async(request)
        future.add_done_callback(self.stop_motor_callback)
    
    def stop_motor_callback(self, future):
        """
        Handle stop_motor service response.
        """
        self.service_call_in_progress = False
        
        try:
            response = future.result()
            self.motor_running = False
            self.last_service_call_time = self.get_clock().now()
            self.get_logger().info('Motor stopped successfully')
        except Exception as e:
            self.get_logger().error(f'stop_motor service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = LidarManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motor on shutdown to save power
        node.get_logger().info('Shutdown: Stopping motor...')
        if node.motor_running:
            node.stop_motor()
            # Give time for service call to complete
            rclpy.spin_once(node, timeout_sec=1.0)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()