# SLAM Mapping - Startup Procedure

Due to rplidar_ros buffer overflow issues in combined launch files, we launch components separately.

## Terminal 1: Driver
```bash
ssh ryan@192.168.86.20
source /opt/ros/humble/setup.bash
source ~/hoverbot/ros2_ws/install/setup.bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

## Terminal 2: RPLidar (wait 5 seconds after driver starts)
```bash
ssh ryan@192.168.86.20
source /opt/ros/humble/setup.bash
source ~/hoverbot/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

## Terminal 3: SLAM Toolbox (wait 3 seconds after RPLidar starts)
```bash
ssh ryan@192.168.86.20
source /opt/ros/humble/setup.bash
source ~/hoverbot/ros2_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=~/hoverbot/ros2_ws/install/hoverbot_bringup/share/hoverbot_bringup/config/slam_toolbox_params.yaml \
  use_sim_time:=false
```

## Terminal 4: Teleop (to drive and build map)
```bash
ssh ryan@192.168.86.20
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Building a Map

1. Start all terminals in order (1, 2, 3, 4)
2. Use teleop to drive around slowly
3. SLAM will build a map as you explore
4. Save map when done (see below)

## Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

This creates `my_map.pgm` and `my_map.yaml`.
