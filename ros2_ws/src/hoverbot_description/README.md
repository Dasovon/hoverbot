# HoverBot Description Package

URDF/Xacro models and Gazebo simulation for the HoverBot robot.

## Contents

```
hoverbot_description/
├── urdf/
│   ├── hoverbot.urdf           # Base robot URDF model
│   └── hoverbot.gazebo.xacro   # Gazebo-enhanced URDF with plugins
├── launch/
│   └── gazebo.launch.py        # Gazebo simulation launch file
└── README.md                   # This file
```

## Robot Specifications

### Physical Dimensions
- **Base**: 60cm (L) × 40cm (W) × 15cm (H)
- **Wheelbase**: 40cm
- **Wheel diameter**: 16.5cm
- **Wheel width**: 6.5cm
- **Total mass**: ~15kg (including sensors)

### Sensors
- **RPLidar A1**: 360° laser scanner
  - Range: 0.15m - 12m
  - Update rate: 7.6 Hz
  - 360 samples per scan

- **BNO055 IMU**: 9-DOF inertial measurement unit
  - Update rate: 100 Hz
  - Outputs: orientation, angular velocity, linear acceleration

- **Intel RealSense D435**: RGB-D camera
  - Resolution: 640×480
  - Update rate: 15 Hz
  - Depth range: 0.1m - 10m

- **ELP Camera**: USB RGB camera
  - Resolution: 1920×1080
  - Update rate: 30 Hz

### Actuators
- **Differential Drive**: 2 motorized wheels
  - Max linear velocity: ~0.26 m/s
  - Max angular velocity: ~1.29 rad/s
  - Control via `/cmd_vel` topic

## Gazebo Simulation

### Prerequisites

Install Gazebo and required ROS 2 packages:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### Launching Gazebo

To start the Gazebo simulation:

```bash
# Build the package first
cd ~/hoverbot/ros2_ws
colcon build --packages-select hoverbot_description
source install/setup.bash

# Launch Gazebo simulation
ros2 launch hoverbot_description gazebo.launch.py
```

### Launch Arguments

You can customize the launch with these arguments:

```bash
# Launch without GUI (headless)
ros2 launch hoverbot_description gazebo.launch.py gui:=false

# Launch with a custom world file
ros2 launch hoverbot_description gazebo.launch.py world:=/path/to/world.world

# Use simulation time
ros2 launch hoverbot_description gazebo.launch.py use_sim_time:=true
```

### Controlling the Robot

Once Gazebo is running, you can control the robot:

```bash
# Using keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish directly to cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### Available Topics

The Gazebo simulation publishes to these topics:

**Sensors:**
- `/scan` - LaserScan from RPLidar A1
- `/imu/data` - IMU data from BNO055
- `/camera/rgb/image_raw` - RGB image from RealSense D435
- `/camera/depth/image_raw` - Depth image from RealSense D435
- `/camera/depth/points` - Point cloud from RealSense D435
- `/elp_camera/image_raw` - RGB image from ELP camera

**Odometry & State:**
- `/odom` - Wheel odometry
- `/joint_states` - Joint positions/velocities
- `/tf` - Transform tree

**Control:**
- `/cmd_vel` - Velocity commands (subscriber)

### Gazebo Plugins

The simulation includes these Gazebo plugins:

1. **Differential Drive Controller** (`libgazebo_ros_diff_drive.so`)
   - Simulates wheel motors and publishes odometry
   - Subscribes to `/cmd_vel`

2. **Ray Sensor** (`libgazebo_ros_ray_sensor.so`)
   - Simulates RPLidar A1 laser scanner

3. **IMU Sensor** (`libgazebo_ros_imu_sensor.so`)
   - Simulates BNO055 IMU with realistic noise

4. **Depth Camera** (`libgazebo_ros_camera.so`)
   - Simulates RealSense D435 RGB-D camera

5. **RGB Camera** (`libgazebo_ros_camera.so`)
   - Simulates ELP USB camera

6. **Joint State Publisher** (`libgazebo_ros_joint_state_publisher.so`)
   - Publishes wheel joint states

## Visualization with RViz2

To visualize the robot in RViz while running Gazebo:

```bash
# In a new terminal
source ~/hoverbot/ros2_ws/install/setup.bash
rviz2
```

Then add displays for:
- **RobotModel** - Set `Description Topic` to `/robot_description`
- **LaserScan** - Topic: `/scan`
- **TF** - Show coordinate frames
- **Image** - Topic: `/camera/rgb/image_raw` or `/elp_camera/image_raw`
- **PointCloud2** - Topic: `/camera/depth/points`

## SLAM in Gazebo

You can run SLAM mapping in simulation:

```bash
# Terminal 1: Start Gazebo
ros2 launch hoverbot_description gazebo.launch.py

# Terminal 2: Start SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true

# Terminal 3: Control the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: View in RViz
rviz2
```

## Navigation in Gazebo

To test autonomous navigation in simulation:

```bash
# Terminal 1: Start Gazebo
ros2 launch hoverbot_description gazebo.launch.py

# Terminal 2: Start Nav2
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true

# Terminal 3: Start RViz with Nav2 config
ros2 launch nav2_bringup rviz_launch.py
```

Then use the "2D Goal Pose" button in RViz to send navigation goals.

## Development

### Modifying the Robot Model

1. Edit `urdf/hoverbot.urdf` for basic robot geometry
2. Edit `urdf/hoverbot.gazebo.xacro` for Gazebo-specific properties
3. Rebuild the package:
   ```bash
   cd ~/hoverbot/ros2_ws
   colcon build --packages-select hoverbot_description
   ```

### Adding New Sensors

To add a new sensor:

1. Add the link and joint to `hoverbot.urdf`
2. Add the Gazebo plugin to `hoverbot.gazebo.xacro`
3. Update this README with the new sensor specifications

## Troubleshooting

### Gazebo won't start
- Ensure Gazebo is installed: `gazebo --version`
- Check that ROS 2 Gazebo packages are installed
- Try starting Gazebo standalone: `gazebo`

### Robot falls through the ground
- Check that the caster wheels have proper collision geometry
- Verify inertial properties are realistic
- Ensure friction coefficients (mu1/mu2) are set

### Robot doesn't move
- Verify `/cmd_vel` messages are being published
- Check Gazebo console for plugin errors
- Ensure wheel joint types are `continuous`

### Sensors not publishing
- Check topic names: `ros2 topic list`
- Verify plugins are loaded: Check Gazebo console output
- Ensure `use_sim_time:=true` is set

## Real Hardware vs Simulation

Key differences between simulation and real hardware:

| Aspect | Real Hardware | Gazebo Simulation |
|--------|---------------|-------------------|
| Odometry | Encoder-based, drift | Perfect (world-based) |
| LiDAR noise | Real-world noise | Gaussian noise model |
| Physics | Real friction, inertia | Approximated |
| Latency | Serial comm delays | Minimal |
| Power | Battery constraints | Unlimited |

**Note**: Simulation is excellent for algorithm development but always test on real hardware before deployment!

## References

- [Gazebo ROS 2 Integration](http://gazebosim.org/tutorials?tut=ros2_overview)
- [URDF in Gazebo](http://gazebosim.org/tutorials?tut=ros_urdf)
- [Gazebo Plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins)
- [ROS 2 Navigation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
