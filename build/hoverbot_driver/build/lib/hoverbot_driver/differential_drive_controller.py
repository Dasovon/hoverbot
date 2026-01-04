"""
Differential Drive Controller

Converts between ROS Twist messages (linear/angular velocity) and
individual wheel commands for the hoverboard motor controller.

Kinematics:
- Robot has two independently driven wheels
- Wheelbase (track width): 0.40 m
- Wheel diameter: 0.165 m
- Tank steering mode: direct left/right wheel control
"""

import math
from typing import Tuple


class DifferentialDriveController:
    """
    Converts Twist commands to wheel velocities and vice versa.
    
    Implements differential drive kinematics for tank steering configuration.
    """
    
    def __init__(
        self,
        wheel_diameter: float = 0.165,  # meters
        wheelbase: float = 0.40,  # meters (track width)
        max_rpm: int = 300,  # maximum wheel RPM
        cmd_range: int = 1000  # firmware command range
    ):
        """
        Initialize differential drive controller.
        
        Args:
            wheel_diameter: Wheel diameter in meters
            wheelbase: Distance between left and right wheels in meters
            max_rpm: Maximum wheel speed in RPM (safety limit)
            cmd_range: Firmware command range (±1000)
        """
        self.wheel_diameter = wheel_diameter
        self.wheel_radius = wheel_diameter / 2.0
        self.wheelbase = wheelbase
        self.max_rpm = max_rpm
        self.cmd_range = cmd_range
        
        # Calculate maximum achievable velocities
        # v_wheel_max = (RPM × 2π × r) / 60
        self.max_wheel_speed = (self.max_rpm * 2.0 * math.pi * self.wheel_radius) / 60.0  # m/s
        
        # Max linear velocity when both wheels at max speed
        self.max_linear_velocity = self.max_wheel_speed  # m/s
        
        # Max angular velocity when wheels at opposite max speeds
        # omega_max = 2 × v_wheel_max / wheelbase
        self.max_angular_velocity = (2.0 * self.max_wheel_speed) / self.wheelbase  # rad/s
    
    def twist_to_wheels(self, linear_x: float, angular_z: float) -> Tuple[int, int]:
        """
        Convert Twist command to individual wheel commands.
        
        Differential drive kinematics:
        v_left  = linear_x - (angular_z × wheelbase / 2)
        v_right = linear_x + (angular_z × wheelbase / 2)
        
        Args:
            linear_x: Forward velocity in m/s (positive = forward)
            angular_z: Angular velocity in rad/s (positive = counter-clockwise)
            
        Returns:
            Tuple of (left_cmd, right_cmd) in firmware units (-1000 to +1000)
        """
        # Calculate wheel velocities in m/s
        half_base = self.wheelbase / 2.0
        v_left = linear_x - (angular_z * half_base)
        v_right = linear_x + (angular_z * half_base)
        
        # Convert m/s to RPM
        # RPM = (v_m/s × 60) / (2π × r)
        rpm_left = (v_left * 60.0) / (2.0 * math.pi * self.wheel_radius)
        rpm_right = (v_right * 60.0) / (2.0 * math.pi * self.wheel_radius)
        
        # Scale RPM to command range
        # cmd = (RPM / max_RPM) × cmd_range
        cmd_left = int((rpm_left / self.max_rpm) * self.cmd_range)
        cmd_right = int((rpm_right / self.max_rpm) * self.cmd_range)
        
        # Clamp to safe limits
        cmd_left = max(-self.cmd_range, min(self.cmd_range, cmd_left))
        cmd_right = max(-self.cmd_range, min(self.cmd_range, cmd_right))
        
        return (cmd_left, cmd_right)
    
    def wheels_to_twist(self, rpm_left: int, rpm_right: int) -> Tuple[float, float]:
        """
        Convert wheel speeds to Twist velocities (for odometry).
        
        Args:
            rpm_left: Left wheel speed in RPM
            rpm_right: Right wheel speed in RPM
            
        Returns:
            Tuple of (linear_x, angular_z) in m/s and rad/s
        """
        # Convert RPM to m/s
        # v = (RPM × 2π × r) / 60
        v_left = (rpm_left * 2.0 * math.pi * self.wheel_radius) / 60.0
        v_right = (rpm_right * 2.0 * math.pi * self.wheel_radius) / 60.0
        
        # Calculate robot velocities
        linear_x = (v_left + v_right) / 2.0
        angular_z = (v_right - v_left) / self.wheelbase
        
        return (linear_x, angular_z)
    
    def get_limits(self) -> dict:
        """
        Get velocity limits for this robot configuration.
        
        Returns:
            Dictionary with max_linear_velocity and max_angular_velocity
        """
        return {
            'max_linear_velocity': self.max_linear_velocity,
            'max_angular_velocity': self.max_angular_velocity,
            'max_wheel_speed': self.max_wheel_speed,
            'max_rpm': self.max_rpm
        }
    
    def validate_twist(self, linear_x: float, angular_z: float) -> Tuple[bool, str]:
        """
        Check if Twist command is within safe limits.
        
        Args:
            linear_x: Requested linear velocity (m/s)
            angular_z: Requested angular velocity (rad/s)
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        # Check linear velocity
        if abs(linear_x) > self.max_linear_velocity:
            return (False, f"Linear velocity {linear_x:.2f} exceeds limit {self.max_linear_velocity:.2f} m/s")
        
        # Check angular velocity
        if abs(angular_z) > self.max_angular_velocity:
            return (False, f"Angular velocity {angular_z:.2f} exceeds limit {self.max_angular_velocity:.2f} rad/s")
        
        # Check if combination results in wheel speeds within limits
        half_base = self.wheelbase / 2.0
        v_left = linear_x - (angular_z * half_base)
        v_right = linear_x + (angular_z * half_base)
        
        if abs(v_left) > self.max_wheel_speed or abs(v_right) > self.max_wheel_speed:
            return (False, f"Commanded twist results in excessive wheel speeds")
        
        return (True, "")
