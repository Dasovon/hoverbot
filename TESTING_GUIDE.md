# HoverBot Testing Guide

## System Ready for Testing ✅

All comprehensive sensor launch files from Pi4 development branch are now on `main` and ready for testing.

---

## On Pi4 (Hardware Testing)

### Prerequisites Check

Ensure these packages are installed:
```bash
# Check ROS 2 Humble is installed
ros2 --version

# Check sensor packages
dpkg -l | grep -E "ros-humble-(rplidar|realsense2-camera|usb-cam|robot-localization|slam-toolbox|bno055)"

# If missing, install:
sudo apt install ros-humble-rplidar-ros \
                 ros-humble-realsense2-camera \
                 ros-humble-usb-cam \
                 ros-humble-robot-localization \
                 ros-humble-slam-toolbox \
                 ros-humble-bno055
```

### Build the Workspace

```bash
cd ~/hoverbot/ros2_ws
colcon build
source install/setup.bash
```

### Option 1: Full System Test (All 7 Sensors)

**What it launches:**
1. Hoverboard driver (odometry @ 50 Hz)
2. BNO055 IMU (orientation @ 20 Hz)
3. RealSense D435 depth camera (@ 30 Hz)
4. ELP USB camera RGB (@ 30 Hz)
5. RPLidar A1 (scan @ 7-10 Hz)
6. Sensor fusion (EKF combining odometry + IMU)
7. SLAM Toolbox (mapping)

**Command:**
```bash
ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py
```

**Startup sequence:**
- [0s]   Hoverboard driver
- [2s]   Static transforms (laser)
- [2.5s] Static transforms (camera)
- [3s]   IMU (BNO055)
- [5s]   Sensor fusion (EKF)
- [8s]   RealSense D435 depth
- [8.5s] ELP USB camera RGB
- [15s]  RPLidar A1
- [17s]  SLAM Toolbox
- **System fully operational in ~19 seconds**

**Expected output:**
```
╔════════════════════════════════════════════════════════════╗
║  HoverBot Full System - V3.1 + Power Management            ║
╚════════════════════════════════════════════════════════════╝

Startup sequence:
  [0s]   Hoverboard driver
  [2s]   Static transforms (laser)
  ...
  [17s]  SLAM Toolbox

System will be fully operational in ~19 seconds.
```

### Option 2: Quick Test (Driver + Lidar Only)

**What it launches:**
- Hoverboard driver
- RPLidar A1

**Command:**
```bash
ros2 launch hoverbot_bringup robot_with_lidar.launch.py
```

### Option 3: Individual Component Testing

**Test hoverboard only:**
```bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

**Test IMU only:**
```bash
ros2 launch hoverbot_bringup imu.launch.py
```

**Test LiDAR with power management:**
```bash
ros2 launch hoverbot_bringup lidar_power_management.launch.py
```

### Platform-Specific Configuration

The Pi4 uses `/dev/ttyAMA0` for the hoverboard. This is configured in:
```
platforms/raspberry-pi4/config/hoverbot_driver.yaml
```

Make sure this config is linked or copied to the ROS 2 workspace.

---

## On Dev Machine (Visualization)

### Prerequisites

```bash
# Install ROS 2 Humble Desktop (includes RViz2)
sudo apt install ros-humble-desktop

# Or just RViz2
sudo apt install ros-humble-rviz2

# Configure ROS domain
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

### Launch RViz2

```bash
# After Pi4 is running the full system
rviz2
```

### RViz2 Configuration

Add these displays:

1. **TF** - Shows robot transforms
   - Fixed Frame: `odom`

2. **LaserScan**
   - Topic: `/scan`
   - Size: 0.05
   - Color: By Intensity

3. **Map**
   - Topic: `/map`
   - Color Scheme: map

4. **Odometry**
   - Topic: `/odom`
   - Shape: Arrow

5. **Camera** (optional)
   - Topic: `/camera/depth/image_rect_raw` (RealSense)
   - Topic: `/elp/image_raw` (ELP RGB)

6. **PointCloud2** (optional)
   - Topic: `/camera/depth/points`

### Monitor Topics

```bash
# Check odometry rate (should be 50 Hz)
ros2 topic hz /odom

# Check LiDAR scan rate (should be 7-10 Hz)
ros2 topic hz /scan

# Check IMU rate (should be 20 Hz)
ros2 topic hz /bno055/imu

# List all topics
ros2 topic list

# Echo odometry
ros2 topic echo /odom
```

### Drive the Robot

```bash
# Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Controls:
#   i - forward
#   , - backward
#   j - rotate left
#   l - rotate right
#   k - stop
```

---

## Verification Checklist

### On Pi4

- [ ] Workspace builds without errors
- [ ] Hoverboard driver connects to `/dev/ttyAMA0`
- [ ] RPLidar connects to `/dev/rplidar` (or `/dev/ttyUSB0`)
- [ ] IMU detected on I2C bus 1, address 0x28
- [ ] RealSense D435 detected via USB
- [ ] ELP camera detected via USB
- [ ] All nodes start without errors
- [ ] No "buffer overflow" errors from LiDAR
- [ ] `/scan` topic publishes at 7-10 Hz
- [ ] `/odom` topic publishes at 50 Hz
- [ ] `/bno055/imu` topic publishes at 20 Hz

### On Dev

- [ ] Can see robot in RViz2
- [ ] TF tree is complete (odom → base_link → laser)
- [ ] LaserScan shows environment
- [ ] Map updates as robot moves
- [ ] Teleop controls work
- [ ] No lag or delays

---

## Troubleshooting

### "Could not open serial port /dev/ttyAMA0"

```bash
# Check if user is in dialout group
sudo usermod -a -G dialout $USER
# Logout and login again

# Check port exists
ls -la /dev/ttyAMA0

# Check permissions
sudo chmod 666 /dev/ttyAMA0
```

### "RPLidar buffer overflow"

This is why we use the 15-second delay in `hoverbot_full_v3.launch.py`. If still occurring:
- Increase delay in launch file
- Use `lidar_power_management.launch.py` which handles this better

### "IMU not detected"

```bash
# Check I2C bus
sudo i2cdetect -y 1

# Should show device at 0x28
# If not, check wiring and I2C is enabled in raspi-config
```

### "RealSense not detected"

```bash
# Check USB connection
lsusb | grep Intel

# Reinstall realsense SDK if needed
sudo apt install ros-humble-realsense2-camera
```

### "Nodes can't see each other"

```bash
# Check ROS_DOMAIN_ID matches on both machines
echo $ROS_DOMAIN_ID  # Should be 0 on both

# Check network connectivity
ping <other-machine-ip>

# Check firewall
sudo ufw allow from <other-machine-ip>
```

---

## Performance Targets

| Metric | Target | Expected |
|--------|--------|----------|
| Serial Success Rate | >95% | 99.3% |
| Odometry Rate | 50 Hz | 50 Hz |
| LiDAR Scan Rate | 10 Hz | 7-10 Hz |
| IMU Rate | 20 Hz | 20 Hz |
| SLAM Map Rate | 0.1 Hz | 0.1 Hz |
| RealSense Depth | 30 Hz | 30 Hz |
| ELP Camera | 30 Hz | 30 Hz |

---

## Next Steps After Testing

1. **If everything works:**
   - Save your map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map`
   - Test autonomous navigation
   - Tune SLAM parameters

2. **If issues found:**
   - Check logs: `ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py 2>&1 | tee test.log`
   - Monitor individual topics
   - Test components separately

3. **Document results:**
   - Update this guide with findings
   - Report issues to GitHub
   - Share performance metrics

---

**Ready to test!** Start with Option 1 (Full System) on Pi4 and RViz2 on Dev.
