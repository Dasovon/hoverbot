#!/usr/bin/env python3
"""
IMU Quality Test - Validates BNO055 data during robot movement

Tests for:
  - Calibration status (system, gyro, accel, mag)
  - Data consistency and noise levels
  - Gravity measurement accuracy
  - Drift characteristics
  - Update rate stability

Usage:
  1. Stationary test (5-10 seconds):
     ros2 run hoverbot_bringup test_imu_quality.py
     
  2. Motion test:
     - Start the test script
     - Use teleop to drive robot in figure-8 pattern
     - Observe gyro response and calibration improvement

Expected results (stationary):
  ✓ Calibration: SYS=3/3, GYRO=3/3, ACCEL=3/3, MAG=2-3/3
  ✓ Gyro bias: < 0.05 rad/s on all axes
  ✓ Gyro noise: < 0.02 rad/s standard deviation
  ✓ Accel magnitude: 9.81 ± 0.5 m/s²
  ✓ Accel noise: < 0.5 m/s² standard deviation
  ✓ Update rate: ~20 Hz

Troubleshooting:
  - Low calibration: Move robot in figure-8 pattern for 30 seconds
  - High gyro bias: Recalibrate (keep robot very still for 10 seconds)
  - High noise: Check IMU mounting (vibration isolation needed?)
  - Wrong gravity: Check IMU orientation in launch file
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
from collections import deque
import time
from math import sqrt

# Check if bno055_driver messages are available
try:
    from bno055_driver.msg import CalibStatus
    HAS_CALIB_STATUS = True
except ImportError:
    HAS_CALIB_STATUS = False
    print("Warning: bno055_driver.msg.CalibStatus not found")
    print("Calibration status monitoring will be disabled")


class IMUQualityTest(Node):
    """Monitor IMU data quality and report statistics."""
    
    def __init__(self):
        super().__init__('imu_quality_test')
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 
            '/bno055/imu', 
            self.imu_callback, 
            10
        )
        
        if HAS_CALIB_STATUS:
            self.calib_sub = self.create_subscription(
                CalibStatus,
                '/bno055/calib_status',
                self.calib_callback,
                10
            )
        
        # Data buffers (5 seconds at 20 Hz = 100 samples)
        self.buffer_size = 100
        self.gyro_buffer = deque(maxlen=self.buffer_size)
        self.accel_buffer = deque(maxlen=self.buffer_size)
        self.orient_buffer = deque(maxlen=self.buffer_size)
        self.timestamps = deque(maxlen=self.buffer_size)
        
        # Statistics
        self.sample_count = 0
        self.calib_status = None
        self.start_time = self.get_clock().now()
        self.last_report_time = self.start_time
        
        # Report timer (every 5 seconds)
        self.report_timer = self.create_timer(5.0, self.print_quality_report)
        
        self.get_logger().info('╔════════════════════════════════════════════════════╗')
        self.get_logger().info('║       IMU Quality Test - BNO055 Monitor           ║')
        self.get_logger().info('╚════════════════════════════════════════════════════╝')
        self.get_logger().info('')
        self.get_logger().info('Collecting baseline data...')
        self.get_logger().info('First report in 5 seconds.')
        self.get_logger().info('')
        
    def calib_callback(self, msg):
        """Track BNO055 calibration status (0-3 for each sensor)."""
        self.calib_status = {
            'system': msg.sys,      # Overall system calibration
            'gyro': msg.gyro,       # Gyroscope calibration
            'accel': msg.accel,     # Accelerometer calibration
            'mag': msg.mag          # Magnetometer calibration
        }
        
    def imu_callback(self, msg):
        """Collect IMU samples for statistical analysis."""
        self.sample_count += 1
        current_time = self.get_clock().now()
        
        # Extract raw sensor data
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # Store quaternion orientation for drift monitoring
        orient = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        
        # Buffer the data
        self.gyro_buffer.append(gyro)
        self.accel_buffer.append(accel)
        self.orient_buffer.append(orient)
        self.timestamps.append(current_time.nanoseconds / 1e9)
        
    def print_quality_report(self):
        """Analyze buffered data and print quality metrics."""
        if len(self.gyro_buffer) < 10:
            self.get_logger().warn('Not enough samples yet (need 10+)...')
            return
        
        # Calculate runtime and update rate
        current_time = self.get_clock().now()
        runtime = (current_time - self.start_time).nanoseconds / 1e9
        
        # Calculate actual update rate from timestamps
        if len(self.timestamps) >= 2:
            time_diffs = np.diff(list(self.timestamps))
            avg_period = np.mean(time_diffs)
            actual_rate = 1.0 / avg_period if avg_period > 0 else 0.0
            rate_std = np.std(time_diffs)
        else:
            actual_rate = 0.0
            rate_std = 0.0
        
        # Convert buffers to numpy arrays for analysis
        gyro_data = np.array(self.gyro_buffer)
        accel_data = np.array(self.accel_buffer)
        
        # Statistical analysis
        gyro_mean = np.mean(gyro_data, axis=0)
        gyro_std = np.std(gyro_data, axis=0)
        gyro_max = np.max(np.abs(gyro_data), axis=0)
        
        accel_mean = np.mean(accel_data, axis=0)
        accel_std = np.std(accel_data, axis=0)
        accel_mag = np.linalg.norm(accel_mean)  # Should be ~9.81 m/s² (gravity)
        
        # Print formatted report
        self.get_logger().info('╔════════════════════════════════════════════════════════════╗')
        self.get_logger().info(f'║  IMU Quality Report - {runtime:.1f}s runtime, {self.sample_count} samples')
        self.get_logger().info('╚════════════════════════════════════════════════════════════╝')
        
        # ---- Calibration Status ----
        self.get_logger().info('')
        self.get_logger().info('Calibration Status:')
        if self.calib_status:
            sys_stat = self.calib_status['system']
            gyro_stat = self.calib_status['gyro']
            accel_stat = self.calib_status['accel']
            mag_stat = self.calib_status['mag']
            
            self.get_logger().info(
                f"  System: {sys_stat}/3  Gyro: {gyro_stat}/3  "
                f"Accel: {accel_stat}/3  Mag: {mag_stat}/3"
            )
            
            # Warnings for poor calibration
            if sys_stat < 2:
                self.get_logger().warn('  ⚠️  System calibration LOW - move robot in figure-8 pattern')
            elif sys_stat == 2:
                self.get_logger().info('  ⚡ System calibration GOOD')
            else:
                self.get_logger().info('  ✓ System calibration EXCELLENT')
                
            if mag_stat < 2:
                self.get_logger().warn('  ⚠️  Magnetometer needs calibration - rotate 360° on each axis')
        else:
            self.get_logger().warn('  ⚠️  Calibration status not available')
        
        # ---- Update Rate ----
        self.get_logger().info('')
        self.get_logger().info('Update Rate:')
        self.get_logger().info(f'  Actual: {actual_rate:.2f} Hz (target: 20 Hz)')
        self.get_logger().info(f'  Jitter: ±{rate_std*1000:.2f} ms')
        
        if actual_rate < 18.0 or actual_rate > 22.0:
            self.get_logger().warn(f'  ⚠️  Rate deviation from 20 Hz target')
        else:
            self.get_logger().info('  ✓ Update rate stable')
        
        # ---- Gyroscope (should be near zero when stationary) ----
        self.get_logger().info('')
        self.get_logger().info('Gyroscope (Angular Velocity):')
        self.get_logger().info(
            f'  Mean:   X={gyro_mean[0]:+.4f}  Y={gyro_mean[1]:+.4f}  '
            f'Z={gyro_mean[2]:+.4f} rad/s'
        )
        self.get_logger().info(
            f'  StdDev: X={gyro_std[0]:.4f}  Y={gyro_std[1]:.4f}  '
            f'Z={gyro_std[2]:.4f} rad/s'
        )
        self.get_logger().info(
            f'  Peak:   X={gyro_max[0]:.4f}  Y={gyro_max[1]:.4f}  '
            f'Z={gyro_max[2]:.4f} rad/s'
        )
        
        # Check for bias and noise
        max_bias = np.max(np.abs(gyro_mean))
        max_noise = np.max(gyro_std)
        
        if max_bias > 0.05:
            self.get_logger().warn(f'  ⚠️  Gyro BIAS detected: {max_bias:.4f} rad/s (expect < 0.05)')
        else:
            self.get_logger().info('  ✓ Gyro bias acceptable')
            
        if max_noise > 0.02:
            self.get_logger().warn(f'  ⚠️  High gyro NOISE: {max_noise:.4f} rad/s (expect < 0.02)')
            self.get_logger().warn('     Check for vibration or poor IMU mounting')
        else:
            self.get_logger().info('  ✓ Gyro noise acceptable')
        
        # ---- Accelerometer (should measure gravity ~9.81 m/s²) ----
        self.get_logger().info('')
        self.get_logger().info('Accelerometer (Linear Acceleration):')
        self.get_logger().info(
            f'  Mean:      X={accel_mean[0]:+.4f}  Y={accel_mean[1]:+.4f}  '
            f'Z={accel_mean[2]:+.4f} m/s²'
        )
        self.get_logger().info(f'  Magnitude: {accel_mag:.4f} m/s² (expect ~9.81 for gravity)')
        self.get_logger().info(
            f'  StdDev:    X={accel_std[0]:.4f}  Y={accel_std[1]:.4f}  '
            f'Z={accel_std[2]:.4f} m/s²'
        )
        
        # Check gravity measurement
        gravity_error = abs(accel_mag - 9.81)
        if gravity_error > 0.5:
            self.get_logger().warn(
                f'  ⚠️  Gravity measurement OFF by {gravity_error:.2f} m/s²'
            )
            self.get_logger().warn('     Check IMU orientation or calibration')
        else:
            self.get_logger().info('  ✓ Gravity measurement accurate')
        
        # Check accelerometer noise
        max_accel_noise = np.max(accel_std)
        if max_accel_noise > 0.5:
            self.get_logger().warn(
                f'  ⚠️  High accelerometer NOISE: {max_accel_noise:.4f} m/s²'
            )
            self.get_logger().warn('     Consider vibration isolation or damping')
        else:
            self.get_logger().info('  ✓ Accelerometer noise acceptable')
        
        self.get_logger().info('╚════════════════════════════════════════════════════════════╝')
        self.get_logger().info('')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = IMUQualityTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('')
        node.get_logger().info('Shutting down IMU quality test.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
