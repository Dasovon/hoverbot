# Quick Start: RealSense D435 - Next Session

**Resume Point:** Camera working, needs integration into robot launch system

---

## Current State ✅

**Hardware:**
- ✅ RealSense D435 connected (S/N: 244622071235)
- ✅ Running on USB 2.1 (blue port, bandwidth limited)
- ✅ Depth camera: 640×480 @ 15fps WORKING
- ⚠️ RGB camera: Limited due to USB bandwidth

**Software:**
- ✅ LibRealSense v2.56.4 installed
- ✅ ROS 2 wrapper installed (ros-humble-realsense2-camera)
- ✅ Camera launches successfully
- ✅ RViz visualization working (via X11)

**Network Setup:**
- Pi IP changed: 192.168.86.20 → 192.168.86.33
- SSH alias: `ssh hoverbot` or `ssh -X hoverbot`
- ROS_DOMAIN_ID: 0

---

## Quick Test (5 min)

### Verify Camera Still Works

**Terminal 1 - Launch Camera:**
```bash
ssh hoverbot
cd ~/hoverbot/ros2_ws
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

**Wait for:** "RealSense Node Is Up!"

**Terminal 2 - Check Topics:**
```bash
ssh hoverbot
export ROS_DOMAIN_ID=0
ros2 topic list | grep camera
ros2 topic hz /camera/camera/depth/image_rect_raw
```

**Expected:** 15 Hz depth updates

---

## View in RViz (10 min)

**Terminal 3 - RViz with X11:**
```bash
ssh -X hoverbot
cd ~/hoverbot/ros2_ws
source install/setup.bash
rviz2
```

**Configure RViz:**
1. Fixed Frame: `camera_link`
2. Add → By topic → `/camera/camera/depth/image_rect_raw` → Image
3. Should see grayscale depth image

---

## Phase 2: Integration Tasks

### Task 1: Add Camera to Launch File (20 min)

**File:** `~/hoverbot/ros2_ws/src/hoverbot_bringup/launch/hoverbot_full_v3.launch.py`

**Add after line 130 (after sensor fusion):**

```python
# ========================================================================
# COMPONENT 6: RealSense D435 Camera (8 second delay)
# Depth camera for enhanced obstacle detection
# ========================================================================
camera_launch = TimerAction(
    period=8.0,
    actions=[
        LogInfo(msg='[8s] Starting RealSense D435 camera...'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                '/opt/ros/humble/share/realsense2_camera/launch/rs_launch.py'
            ]),
            launch_arguments={
                'depth_module.depth_profile': '640x480x15',
                'rgb_camera.color_profile': '424x240x6',  # Low-res RGB
                'enable_sync': 'true',
                'align_depth.enable': 'true'
            }.items()
        )
    ]
)
```

**Add to return statement:**
```python
return LaunchDescription([
    # ... existing components ...
    camera_launch,  # Add this line
    # ... rest of launch ...
])
```

**Update startup message (around line 205):**
```python
LogInfo(msg='  [8s]  RealSense D435 camera'),
```

### Task 2: Add Camera Transform (10 min)

**Add after base_to_laser_tf (around line 95):**

```python
# Camera transform (base_link → camera_link)
TimerAction(
    period=2.5,
    actions=[
        LogInfo(msg='[2.5s] Starting camera transform...'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', '0.15',      # Camera 15cm forward of base_link
                '--y', '0.0',       # Centered
                '--z', '0.20',      # Camera 20cm above base_link
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ],
            output='screen'
        )
    ]
)
```

### Task 3: Test Complete System (15 min)

**Launch everything:**
```bash
cd ~/hoverbot/ros2_ws
colcon build --packages-select hoverbot_bringup
source install/setup.bash

# Make sure hoverboard is powered ON!
ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py
```

**Expected startup sequence:**
```
[0s]  Hoverboard driver
[2s]  Static transforms (laser)
[2.5s] Static transforms (camera)
[3s]  IMU (BNO055)
[5s]  Sensor fusion (EKF)
[8s]  RealSense D435 camera
[15s] RPLidar A1
[17s] SLAM Toolbox
──────────────────────────
SYSTEM READY ✓
```

**Verify all nodes running:**
```bash
ros2 node list
```

**Expected nodes (8 total):**
- /hoverbot_driver
- /base_to_laser_tf
- /base_to_camera_tf
- /imu
- /ekf_filter_node
- /camera/camera
- /rplidar_node
- /slam_toolbox

### Task 4: Update RViz Config (10 min)

**Open RViz:**
```bash
ssh -X hoverbot
rviz2
```

**Add displays:**
1. Fixed Frame: `odom`
2. LaserScan: `/scan` (red dots)
3. TF: Coordinate frames
4. Map: `/map`
5. Image: `/camera/camera/depth/image_rect_raw` (depth)
6. Odometry: `/odometry/filtered` (robot pose)

**Save config:**
- File → Save Config As
- Save to: `~/hoverbot/config/hoverbot_complete.rviz`

---

## Troubleshooting

### Camera Doesn't Launch

**Check:**
```bash
# Is camera plugged in?
lsusb | grep Intel

# Check launch file syntax
cd ~/hoverbot/ros2_ws/src/hoverbot_bringup/launch
python3 -m py_compile hoverbot_full_v3.launch.py
```

### Transform Errors

**Symptoms:** "Transform from X to Y does not exist"

**Fix:**
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Verify camera transform published
ros2 topic echo /tf_static | grep camera
```

### High CPU Usage

**Monitor:**
```bash
htop
```

**Expected usage:**
- Hoverboard: ~5%
- RPLidar: ~8%
- IMU: ~3%
- EKF: ~3%
- SLAM: ~12%
- Camera: ~15%
- **Total: ~50-60%** (acceptable)

**If >80%:**
- Reduce camera resolution
- Disable RGB camera
- Lower SLAM update rate

---

## Success Criteria

**Phase 2 Complete When:**
- ✅ Camera launches with robot system
- ✅ All transforms connected (odom → base_link → camera_link)
- ✅ RViz shows: lidar + depth + map + TF
- ✅ CPU usage <80%
- ✅ No launch errors
- ✅ All topics publishing at expected rates

---

## Commands Reference

### Launch Commands
```bash
# Full system (with camera)
ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py

# Camera only (testing)
ros2 launch realsense2_camera rs_launch.py

# RViz with saved config
rviz2 -d ~/hoverbot/config/hoverbot_complete.rviz
```

### Topic Checks
```bash
# List all topics
ros2 topic list

# Check rates
ros2 topic hz /scan /odom /camera/camera/depth/image_rect_raw

# Monitor TF
ros2 run tf2_ros tf2_echo odom camera_link
```

### Node Management
```bash
# List nodes
ros2 node list

# Node details
ros2 node info /camera/camera

# Kill everything
killall -9 ros2 rplidarNode async_slam_toolbox_node
```

---

## Next Phase: Depth-Based Navigation

**After Phase 2 Complete:**

1. **Configure Nav2 with depth layer**
   - Add depth as obstacle source
   - Tune costmap parameters
   - Test autonomous navigation

2. **Point cloud processing**
   - Filter ground plane
   - Detect 3D obstacles
   - Combine with 2D lidar

3. **Advanced features**
   - Visual odometry (if RGB works)
   - Object detection
   - Person following

---

**Files to Reference:**
- Full guide: `REALSENSE_D435_INTEGRATION.md`
- V3 docs: `~/hoverbot/docs/v3/`
- Firmware guide: `~/hoverbot/docs/firmware/BOARD_VARIANT_DEEP_DIVE.md`

**Ready to start Phase 2!** 🚀
