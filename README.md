# Hoverbot2

ROS2 Humble autonomous hoverboard robot

## Hardware
- Raspberry Pi 4 (will upgrade to Pi5)
- RPLidar A1
- BNO055 IMU
- RealSense Camera
- ELP 2MP Camera
- Hoverboard motors + controller

## Workspace Structure
- **Dev machine**: Ubuntu x86_64 - development & visualization
- **Pi (hoverbot)**: Hardware control and sensor integration

## Current Packages
- `hoverbot_bringup` - Launch files and configuration
- `hoverbot_description` - URDF and robot model
- `hoverbot_driver` - Motor control
- `rplidar_ros` - Lidar driver
- `bno055` - IMU driver (in progress)

## Status
- ✅ ROS2 Humble installed (binary)
- ✅ RPLidar A1 working
- ✅ Network communication (ROS_DOMAIN_ID=42)
- 🔧 BNO055 IMU (driver issues)
- 📋 Hoverboard control TODO
