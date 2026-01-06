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

## Editor / Python setup

Recommended workflow to make VS Code/Pylance resolve ROS and workspace packages:

1. Select the Python interpreter used with ROS (typically Python 3.10 for ROS2 Humble) via the Command Palette: `Python: Select Interpreter`.

2. Source ROS and your workspace in a terminal (or add to your shell rc):

```bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash
```

3. Verify imports from the active interpreter:

```bash
python3 -c "import rclpy, sensor_msgs; print('rclpy & sensor_msgs import OK')"
```

4. If editor warnings persist, restart the window / Pylance: Command Palette → `Developer: Reload Window`.

This repository also includes a `.vscode/settings.json` with `python.analysis.extraPaths` to help Pylance find ROS and workspace packages.
