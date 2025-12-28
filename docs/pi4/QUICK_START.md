# SLAM Quick Start - Pi 4

## Pre-Flight Checklist
- [ ] Raspberry Pi 4 powered on
- [ ] RPLidar USB plugged into Pi
- [ ] Hoverboard battery connected and charged
- [ ] Hoverboard powered on (button pressed, latched)
- [ ] SSH connection working: `ssh ryan@192.168.86.20`

## 5-Terminal Startup (Copy-Paste Ready)

### Terminal 1: Driver
```bash
ssh ryan@192.168.86.20 && source /opt/ros/humble/setup.bash && source ~/hoverbot/ros2_ws/install/setup.bash && ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

### Terminal 2: Transform
```bash
ssh ryan@192.168.86.20 && source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher --x 0.1 --y 0 --z 0.1 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id laser
```

### Terminal 3: RPLidar
```bash
ssh ryan@192.168.86.20 && source /opt/ros/humble/setup.bash && ros2 launch rplidar_ros rplidar_a1_launch.py
```

### Terminal 4: SLAM
```bash
ssh ryan@192.168.86.20 && source /opt/ros/humble/setup.bash && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/ryan/hoverbot/ros2_ws/install/hoverbot_bringup/share/hoverbot_bringup/config/slam_toolbox_params.yaml use_sim_time:=false
```

### Terminal 5: Teleop
```bash
ssh ryan@192.168.86.20 && source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Quick Commands

**Save map:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map_$(date +%Y%m%d_%H%M%S)
```

**Check system:**
```bash
ros2 topic hz /odom   # Should be ~50 Hz
ros2 topic hz /scan   # Should be ~7 Hz
ros2 topic hz /map    # Should be ~0.1 Hz
```

## Shutdown

1. Ctrl+C in all 5 terminals
2. Unplug RPLidar USB
3. Power off hoverboard
4. Shutdown Pi (optional): `sudo shutdown now`
