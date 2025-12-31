# SLAM Testing Results - December 27, 2024

## Test Configuration

**Hardware:**
- Raspberry Pi 4 (4GB)
- Ubuntu 22.04 LTS Server
- ROS 2 Humble Hawksbill
- Hoverboard with EFeru firmware
- RPLidar A1

**Network:**
- Pi IP: 192.168.86.20
- Hostname: hoverbot

**Test Type:** Bench test (RPLidar and hoverboard NOT physically connected)

## Test Results Summary

### ✅ Working Components

1. **Hoverboard Driver**
   - Serial communication: ✅ Working (99.3% success rate)
   - Odometry publishing: ✅ 50 Hz
   - TF broadcasting: ✅ odom → base_link
   - Teleop control: ✅ Wheels respond to commands
   - cmd_vel timeout safety: ✅ Working correctly

2. **RPLidar A1**
   - USB connection: ✅ Detected as /dev/ttyUSB0
   - Health status: ✅ OK
   - Scan rate: ✅ ~7 Hz (10 Hz expected)
   - Scan topic: ✅ Publishing /scan
   - Range: ✅ 12m max, 8 KHz sample rate

3. **Static Transform Publisher**
   - Transform: ✅ base_link → laser (0.1m forward, 0.1m up)
   - Publishing: ✅ /tf_static at 10000 Hz

4. **SLAM Toolbox**
   - Initialization: ✅ Registered sensor successfully
   - Map publishing: ✅ /map at 0.1 Hz (every 10 seconds)
   - Transform chain: ✅ map → odom → base_link → laser
   - Map saving: ✅ Successfully saved multiple maps
   - Map growth: ✅ Observed (40x68 → 74x58 pixels)

### ⚠️ Known Issues

1. **RPLidar Buffer Overflow**
   - Combined launch files cause buffer overflow crash
   - Workaround: Launch components separately in sequence
   - Issue: Known bug in rplidar_ros with ROS 2 Humble
   - Impact: Must use 5-terminal startup sequence

2. **SLAM Initial Message Dropping**
   - First few seconds show "Message Filter dropping message"
   - Resolves automatically after SLAM initialization
   - Impact: None - normal behavior

3. **Map Topic Delay**
   - Maps publish every 10 seconds
   - map_saver_cli may timeout if not timed correctly
   - Workaround: Run map_saver and wait for next publish cycle

### 📊 Performance Metrics

| Component | Expected | Actual | Status |
|-----------|----------|--------|--------|
| Odometry rate | 50 Hz | 50 Hz | ✅ |
| Scan rate | 10 Hz | 7 Hz | ⚠️ Lower |
| Map rate | 0.1 Hz | 0.1 Hz | ✅ |
| Serial success | >95% | 99.3% | ✅ |
| TF broadcast | 50 Hz | 50 Hz | ✅ |

## Startup Sequence (5 Terminals)

### Terminal 1: Driver
```bash
ssh ryan@192.168.86.20
source /opt/ros/humble/setup.bash
source ~/hoverbot/ros2_ws/install/setup.bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```
Wait for: `HoverBot driver node started`

### Terminal 2: Static Transform
```bash
ssh ryan@192.168.86.20
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.1 --y 0 --z 0.1 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser
```
Wait for: `Spinning until stopped - publishing transform`

### Terminal 3: RPLidar
```bash
ssh ryan@192.168.86.20
source /opt/ros/humble/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```
Wait for: `RPLidar health status : OK`

### Terminal 4: SLAM Toolbox
```bash
ssh ryan@192.168.86.20
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/ryan/hoverbot/ros2_ws/install/hoverbot_bringup/share/hoverbot_bringup/config/slam_toolbox_params.yaml \
  use_sim_time:=false
```
Wait for: `Registering sensor: [Custom Described Lidar]`

### Terminal 5: Teleop
```bash
ssh ryan@192.168.86.20
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Saving Maps
```bash
# In any terminal
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# Creates:
# - my_map.pgm (grayscale image)
# - my_map.yaml (metadata)

# Copy to dev machine:
scp ryan@192.168.86.20:~/my_map.* ~/Downloads/
```

## Hardware Requirements for Actual Mapping

**Critical:** For SLAM to work correctly:
- ✅ Hoverboard must be powered on
- ✅ RPLidar must be USB connected to Pi
- ⚠️ **RPLidar must be physically mounted on robot base**
- ⚠️ **Robot must move for accurate mapping**

**Bench test limitations:**
- RPLidar and hoverboard are separate
- Wheels spin but lidar doesn't move with them
- SLAM sees stationary environment while odometry changes
- Creates inconsistent maps (expected behavior)

## Next Steps

1. **Physical Assembly**
   - Mount RPLidar on hoverbot chassis
   - Secure wiring
   - Verify clearances for wheel movement

2. **Real SLAM Testing**
   - Drive robot around actual space
   - Build complete room maps
   - Test loop closure
   - Verify map accuracy

3. **Nav2 Integration**
   - Add autonomous navigation
   - Configure costmaps
   - Test path planning
   - Add obstacle avoidance

4. **Startup Script**
   - Create tmux/screen script for easier launching
   - Add automatic sourcing
   - Error checking and recovery

## Test Date
December 27, 2024

## Tested By
Ryan

## Status
✅ All software components validated
⏭️ Ready for physical assembly and real-world testing
