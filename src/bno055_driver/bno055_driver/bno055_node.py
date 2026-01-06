#!/usr/bin/env python3
"""
BNO055 IMU Driver Node using Adafruit CircuitPython library
Publishes sensor_msgs/Imu, MagneticField, and Temperature
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header
import board
import adafruit_bno055
from math import radians


class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_imu')
        
        # Declare parameters
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # Get parameters
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value
        
        # Initialize I2C and sensor
        try:
            i2c = board.I2C()
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            self.get_logger().info('BNO055 sensor initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO055: {e}')
            raise
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temp', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / rate, self.publish_data)
        
        self.get_logger().info(f'BNO055 node started at {rate} Hz')
        self.get_logger().info(f'Publishing on topics: /imu/data, /imu/mag, /imu/temp')
    
    def publish_data(self):
        """Read sensor and publish all data"""
        try:
            # Create timestamp
            stamp = self.get_clock().now().to_msg()
            
            # Publish IMU data
            self.publish_imu(stamp)
            
            # Publish magnetometer data
            self.publish_magnetometer(stamp)
            
            # Publish temperature (at lower rate)
            if self.get_clock().now().nanoseconds % 1_000_000_000 < 20_000_000:  # ~1Hz
                self.publish_temperature(stamp)
            
        except Exception as e:
            self.get_logger().warn(f'Error reading sensor: {e}')
    
    def publish_imu(self, stamp):
        """Publish IMU message (orientation, angular velocity, linear acceleration)"""
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        
        # Orientation (quaternion)
        quat = self.sensor.quaternion
        if quat[0] is not None:  # Check if data is valid
            imu_msg.orientation.w = quat[0]
            imu_msg.orientation.x = quat[1]
            imu_msg.orientation.y = quat[2]
            imu_msg.orientation.z = quat[3]
            # Orientation covariance (adjust based on calibration)
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01
        else:
            # No valid orientation data
            imu_msg.orientation_covariance[0] = -1
        
        # Angular velocity (gyroscope)
        gyro = self.sensor.gyro
        if gyro[0] is not None:
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]
            # Gyro covariance
            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.01
        else:
            imu_msg.angular_velocity_covariance[0] = -1
        
        # Linear acceleration
        accel = self.sensor.linear_acceleration
        if accel[0] is not None:
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]
            # Accel covariance
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01
        else:
            imu_msg.linear_acceleration_covariance[0] = -1
        
        self.imu_pub.publish(imu_msg)
    
    def publish_magnetometer(self, stamp):
        """Publish magnetometer data"""
        mag_msg = MagneticField()
        mag_msg.header = Header()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self.frame_id
        
        mag = self.sensor.magnetic
        if mag[0] is not None:
            # Convert from µT to T (Tesla)
            mag_msg.magnetic_field.x = mag[0] * 1e-6
            mag_msg.magnetic_field.y = mag[1] * 1e-6
            mag_msg.magnetic_field.z = mag[2] * 1e-6
            
            # Magnetometer covariance
            mag_msg.magnetic_field_covariance[0] = 0.01
            mag_msg.magnetic_field_covariance[4] = 0.01
            mag_msg.magnetic_field_covariance[8] = 0.01
        
        self.mag_pub.publish(mag_msg)
    
    def publish_temperature(self, stamp):
        """Publish temperature data"""
        temp_msg = Temperature()
        temp_msg.header = Header()
        temp_msg.header.stamp = stamp
        temp_msg.header.frame_id = self.frame_id
        
        temp = self.sensor.temperature
        if temp is not None:
            temp_msg.temperature = float(temp)
            temp_msg.variance = 0.5  # Approximate
        
        self.temp_pub.publish(temp_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BNO055Node()
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