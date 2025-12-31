#!/usr/bin/env python3
"""
HoverBot Driver Node

Main ROS 2 node that integrates all components:
- Subscribes to /cmd_vel (Twist commands from navigation)
- Publishes /odom (odometry from wheel encoders)
- Broadcasts TF transform (odom → base_link)
- Publishes /diagnostics (battery, temperature, errors)
- Maintains 50Hz heartbeat to prevent hoverboard timeout

Node runs at 50Hz to satisfy firmware timeout requirements (160ms default).
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import math
import time

from .serial_interface import HoverboardSerialInterface
from .differential_drive_controller import DifferentialDriveController


class HoverBotDriverNode(Node):
    """
    ROS 2 driver node for EFeru hoverboard-based robot.
    
    Responsibilities:
    - Command translation (Twist → motor commands)
    - Odometry estimation (wheel speeds → pose)
    - Transform broadcasting (odom → base_link)
    - Heartbeat management (prevents timeout beeping)
    - Diagnostic publishing (battery, temperature, comm status)
    """
    
    def __init__(self):
        super().__init__('hoverbot_driver')
        
        # Declare parameters with defaults
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_diameter', 0.165)  # meters
        self.declare_parameter('wheelbase', 0.40)  # meters
        self.declare_parameter('max_rpm', 300)
        self.declare_parameter('cmd_vel_timeout', 0.5)  # seconds
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        wheel_diameter = self.get_parameter('wheel_diameter').value
        wheelbase = self.get_parameter('wheelbase').value
        max_rpm = self.get_parameter('max_rpm').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.publish_odom_flag = self.get_parameter('publish_odom').value
        self.publish_tf_flag = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Initialize components
        self.serial = HoverboardSerialInterface(port=serial_port, baudrate=baud_rate)
        self.controller = DifferentialDriveController(
            wheel_diameter=wheel_diameter,
            wheelbase=wheelbase,
            max_rpm=max_rpm
        )
        
        # State variables
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_received = False
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Odometry state
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.theta = 0.0  # radians
        self.last_odom_time = self.get_clock().now()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.activity_pub = self.create_publisher(Bool, '/robot_active', 10) 

        # Activity message
        self.activity_msg = Bool() 
        
        # TF broadcaster
        if self.publish_tf_flag:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Connect to serial port
        if not self.serial.connect():
            self.get_logger().error(f"Failed to connect to serial port {serial_port}")
            raise RuntimeError("Serial connection failed")
        
        self.get_logger().info(f"Connected to hoverboard on {serial_port} at {baud_rate} baud")
        
        # Log velocity limits
        limits = self.controller.get_limits()
        self.get_logger().info(f"Robot limits:")
        self.get_logger().info(f"  Max linear velocity: {limits['max_linear_velocity']:.2f} m/s")
        self.get_logger().info(f"  Max angular velocity: {limits['max_angular_velocity']:.2f} rad/s")
        self.get_logger().info(f"  Max wheel speed: {limits['max_wheel_speed']:.2f} m/s ({limits['max_rpm']} RPM)")
        
        # Main control loop at 50Hz (required for heartbeat)
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz = 20ms
        
        self.get_logger().info("HoverBot driver node started")
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Handle incoming velocity commands from navigation stack.
        
        Args:
            msg: Twist message with linear.x and angular.z
        """
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_received = True

        # Signal robot is active (for LiDAR power management)
        self.activity_msg.data = True
        self.activity_pub.publish(self.activity_msg)
    
    def control_loop(self):
        """
        Main control loop - runs at 50Hz.
        
        Executes:
        1. Check cmd_vel timeout
        2. Convert Twist → wheel commands
        3. Send commands to hoverboard
        4. Read feedback telemetry
        5. Update odometry
        6. Publish odometry and TF
        7. Publish diagnostics
        """
        # Check for cmd_vel timeout
        time_since_cmd = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        
        if time_since_cmd > self.cmd_vel_timeout:
            # Timeout - send zero command
            linear_x = 0.0
            angular_z = 0.0
            if self.cmd_vel_received:
                self.get_logger().warn("cmd_vel timeout - stopping robot")
                self.cmd_vel_received = False

                # Signal robot is idle (stop LiDAR motor)
                self.activity_msg.data = False
                self.activity_pub.publish(self.activity_msg)
        else:
            # Use current commanded velocity
            linear_x = self.current_linear
            angular_z = self.current_angular
        
        # Convert Twist to wheel commands
        left_cmd, right_cmd = self.controller.twist_to_wheels(linear_x, angular_z)
        
        # Send command to hoverboard (TANK_STEERING: left=steer, right=speed)
        success = self.serial.send_command(left_cmd, right_cmd)
        
        if not success:
            self.get_logger().error("Failed to send command to hoverboard")
        
        # Read feedback telemetry
        feedback = self.serial.read_feedback()
        
        if feedback is not None:
            # Update odometry from wheel speeds
            self.update_odometry(feedback.speed_l_rpm, feedback.speed_r_rpm)
            
            # Publish diagnostics every 10 cycles (5Hz)
            if self.serial.rx_count % 10 == 0:
                self.publish_diagnostics(feedback)
    
    def update_odometry(self, rpm_left: int, rpm_right: int):
        """
        Update robot pose from wheel encoder feedback.
        
        Uses dead reckoning with differential drive kinematics.
        
        Args:
            rpm_left: Left wheel speed in RPM
            rpm_right: Right wheel speed in RPM
        """
        # Get current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time
        
        # Convert wheel speeds to robot velocities
        linear_x, angular_z = self.controller.wheels_to_twist(rpm_left, rpm_right)
        
        # Update pose (simple Euler integration)
        # For small dt, this is reasonably accurate
        delta_theta = angular_z * dt
        delta_x = linear_x * math.cos(self.theta + delta_theta / 2.0) * dt
        delta_y = linear_x * math.sin(self.theta + delta_theta / 2.0) * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry message
        if self.publish_odom_flag:
            self.publish_odometry(current_time, linear_x, angular_z)
        
        # Publish TF transform
        if self.publish_tf_flag:
            self.publish_transform(current_time)
    
    def publish_odometry(self, timestamp, linear_x: float, angular_z: float):
        """
        Publish odometry message.
        
        Args:
            timestamp: ROS time for the message
            linear_x: Current linear velocity (m/s)
            angular_z: Current angular velocity (rad/s)
        """
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation = self.quaternion_from_yaw(self.theta)
        
        # Velocity
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_z
        
        # Covariance (rough estimates - tune based on testing)
        # Pose covariance (x, y, z, roll, pitch, yaw)
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.05  # yaw
        
        # Twist covariance (vx, vy, vz, vroll, vpitch, vyaw)
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.05  # vyaw
        
        self.odom_pub.publish(odom)
    
    def publish_transform(self, timestamp):
        """
        Broadcast TF transform from odom to base_link.
        
        Args:
            timestamp: ROS time for the transform
        """
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Rotation
        t.transform.rotation = self.quaternion_from_yaw(self.theta)
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_diagnostics(self, feedback):
        """
        Publish diagnostic information.
        
        Args:
            feedback: HoverboardFeedback object
        """
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Hoverboard status
        status = DiagnosticStatus()
        status.name = "HoverBoard Controller"
        status.hardware_id = "hoverboard_uart"
        
        # Battery voltage (raw value × 100)
        battery_v = feedback.bat_voltage / 100.0
        
        # Temperature (raw value × 10)
        temp_c = feedback.board_temp / 10.0
        
        # Determine status level
        if battery_v < 30.0:
            status.level = DiagnosticStatus.WARN
            status.message = "Low battery voltage"
        elif temp_c > 60.0:
            status.level = DiagnosticStatus.WARN
            status.message = "High temperature"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Operating normally"
        
        # Add diagnostic values
        stats = self.serial.get_stats()
        status.values = [
            KeyValue(key="Battery Voltage", value=f"{battery_v:.2f} V"),
            KeyValue(key="Temperature", value=f"{temp_c:.1f} °C"),
            KeyValue(key="Speed Left", value=f"{feedback.speed_l_rpm} RPM"),
            KeyValue(key="Speed Right", value=f"{feedback.speed_r_rpm} RPM"),
            KeyValue(key="TX Count", value=str(stats['tx_count'])),
            KeyValue(key="RX Count", value=str(stats['rx_count'])),
            KeyValue(key="Checksum Errors", value=str(stats['checksum_errors'])),
            KeyValue(key="Framing Errors", value=str(stats['framing_errors']))
        ]
        
        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)
    
    @staticmethod
    def quaternion_from_yaw(yaw: float) -> Quaternion:
        """
        Convert yaw angle to quaternion.
        
        Args:
            yaw: Rotation around Z-axis in radians
            
        Returns:
            Quaternion message
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down HoverBot driver")
        # Send zero command before disconnecting
        self.serial.send_command(0, 0)
        time.sleep(0.1)
        self.serial.disconnect()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = HoverBotDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
