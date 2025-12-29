# HoverBot V3 Installation Guide

Complete installation for fixed launch system, sensor fusion, and IMU quality testing.

**What's New in V3:**
- ✅ Fixed serial port scoping bug (no more tmux required!)
- ✅ Sensor fusion with robot_localization
- ✅ IMU quality monitoring tool
- ✅ Clean startup sequence with proper timing
- ✅ Better diagnostics and logging

---

## Prerequisites

Verify these packages are installed:

```bash
# On Raspberry Pi
sudo apt update
sudo apt install -y \
    ros-humble-robot-localization \
    ros-humble-teleop-twist-keyboard
```

---

## Installation Steps

### 1. Copy Launch File

```bash
# On your dev machine or Pi
cd ~/hoverbot/ros2_ws/src/hoverbot_bringup/launch
cp /tmp/hoverbot_full_v3.launch.py .

# Make backup of old version
mv hoverbot_full_v2.launch.py hoverbot_full_v2.launch.py.backup
```

### 2. Copy EKF Configuration

```bash
# Create config directory if it doesn't exist
mkdir -p ~/hoverbot/ros2_ws/src/hoverbot_bringup/config

# Copy EKF config
cp /tmp/ekf.yaml ~/hoverbot/ros2_ws/src/hoverbot_bringup/config/
```

### 3. Copy IMU Test Script

```bash
# Create scripts directory if it doesn't exist
mkdir -p ~/hoverbot/ros2_ws/src/hoverbot_bringup/scripts

# Copy test script
cp /tmp/test_imu_quality.py ~/hoverbot/ros2_ws/src/hoverbot_bringup/scripts/

# Make executable
chmod +x ~/hoverbot/ros2_ws/src/hoverbot_bringup/scripts/test_imu_quality.py
```

### 4. Update Shutdown Script

```bash
# Replace old shutdown script
cp /tmp/hoverbot_shutdown.sh ~/hoverbot/scripts/
chmod +x ~/hoverbot/scripts/hoverbot_shutdown.sh
```

### 5. Update CMakeLists.txt (for test script)

Add the test script to your `hoverbot_bringup` package:

```bash
cd ~/hoverbot/ros2_ws/src/hoverbot_bringup
```

Edit `CMakeLists.txt` and add:

```cmake
# Install Python scripts
install(PROGRAMS
  scripts/test_imu_quality.py
  DESTINATION lib/${PROJECT_NAME}
)
```

### 6. Rebuild Workspace

```bash
cd ~/hoverbot/ros2_ws
colcon build --packages-select hoverbot_bringup
source install/setup.bash
```

---

## Testing

### Test 1: Individual Components (Verify Nothing Broke)

```bash
# Terminal 1: Driver
ros2 launch hoverbot_driver hoverbot_driver.launch.py

# Terminal 2: IMU
ros2 launch hoverbot_bringup imu.launch.py

# Verify both are working before proceeding
```

### Test 2: Combined Launch (The Fix!)

```bash
# Single terminal - this should work perfectly now!
ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py
```

**Expected output:**
```
[0s]  Hoverboard driver
[2s]  Static transforms
[3s]  IMU (BNO055)
[5s]  Sensor fusion (EKF)
[8s]  RPLidar A1
[10s] SLAM Toolbox

SYSTEM READY ✓
```

**Verify topics:**
```bash
# In another terminal
ros2 topic list

# You should see:
#   /odom                    (from hoverboard driver)
#   /odometry/filtered       (from EKF - NEW!)
#   /scan                    (from RPLidar)
#   /bno055/imu             (from IMU)
#   /cmd_vel                (command input)
```

### Test 3: IMU Quality Test

```bash
# Keep the system running (hoverbot_full_v3.launch.py)

# In another terminal:
ros2 run hoverbot_bringup test_imu_quality.py

# Let it run for 10-15 seconds, observe calibration status
# Then try driving with teleop to see motion response
```

### Test 4: Sensor Fusion Verification

```bash
# Monitor both odometry sources
ros2 topic echo /odom                # Raw wheel odometry
ros2 topic echo /odometry/filtered   # Fused estimate (NEW!)

# Drive robot and compare - fused should be smoother
```

---

## Troubleshooting

### Issue: "serial_port parameter not set correctly"

**Cause:** Old cached parameter files

**Fix:**
```bash
# Clear parameter cache
rm -rf ~/hoverbot/ros2_ws/install/hoverbot_*/share/hoverbot_*/config/.*.yaml
colcon build --packages-select hoverbot_bringup --cmake-clean-cache
```

### Issue: RPLidar buffer overflow

**Symptoms:** `mismatch packet` errors

**Fix:** The 8-second delay should handle this, but if it persists:
```bash
# Edit hoverbot_full_v3.launch.py
# Change line: period=8.0  →  period=10.0
```

### Issue: EKF not publishing

**Check:**
```bash
# Verify dependencies
ros2 pkg list | grep robot_localization

# Check EKF diagnostics
ros2 topic echo /diagnostics

# Enable debug mode in ekf.yaml:
# debug: true
```

### Issue: IMU test script fails

**Error:** `ModuleNotFoundError: No module named 'bno055_driver'`

**Fix:**
```bash
# The CalibStatus message might not be installed
# Edit test_imu_quality.py and set:
# HAS_CALIB_STATUS = False

# Or install the message package:
sudo apt install ros-humble-bno055-driver
```

---

## Configuration Tuning

### Adjust Sensor Fusion Weights

Edit `~/hoverbot/ros2_ws/src/hoverbot_bringup/config/ekf.yaml`:

**Trust wheel odometry more (better floor):**
```yaml
odom0_twist_covariance: [
  0.0005, ...  # Reduce vx covariance (was 0.001)
```

**Trust IMU orientation more (good calibration):**
```yaml
imu0_pose_covariance: [
  ..., 0.0005, ...  # Reduce yaw covariance (was 0.005)
```

**Faster response (more dynamic robot):**
```yaml
process_noise_covariance: [
  0.1, ...  # Increase from 0.05 for position states
```

### Change Startup Timing

Edit `hoverbot_full_v3.launch.py` period values:

```python
# Component delays (in seconds)
tf_static:      period=2.0    # Can reduce to 1.0
imu_launch:     period=3.0    # Can reduce to 2.0
sensor_fusion:  period=5.0    # Keep >= imu_launch + 2s
rplidar_launch: period=8.0    # CRITICAL: Don't reduce below 8s
slam_launch:    period=10.0   # Can reduce to 9.0
```

---

## Next Steps

1. **Test autonomous navigation:**
   ```bash
   ros2 launch nav2_bringup navigation_launch.py
   ```

2. **Calibrate IMU mounting orientation:**
   - Drive robot, check if IMU yaw matches odometry yaw
   - Adjust static transform if needed

3. **Save your first map:**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/my_first_map
   ```

4. **Update repository:**
   ```bash
   cd ~/hoverbot
   git add .
   git commit -m "V3: Fixed launch bug, added sensor fusion + IMU testing"
   git push origin pi4
   ```

---

## What Changed (Technical Summary)

### 1. Launch File Fix (hoverbot_full_v3.launch.py)

**Before (broken):**
```python
serial_port = LaunchConfiguration('serial_port')  # Ambiguous!
driver_launch = IncludeLaunchDescription(...)     # Inherits wrong value
```

**After (fixed):**
```python
hoverboard_port = LaunchConfiguration('hoverboard_port')  # Explicit
lidar_port = LaunchConfiguration('lidar_port')            # Explicit

driver_launch = IncludeLaunchDescription(...,
    launch_arguments={'serial_port': hoverboard_port}.items()  # Explicit pass
)
```

### 2. Sensor Fusion (ekf.yaml)

Implements Extended Kalman Filter (EKF) that fuses:
- **Odometry:** Provides velocity estimates (vx, vy, vyaw)
- **IMU:** Provides orientation (roll, pitch, yaw) and angular velocity

Output: `/odometry/filtered` with reduced drift and better orientation.

### 3. IMU Quality Test (test_imu_quality.py)

Monitors:
- Update rate (should be ~20 Hz)
- Calibration status (0-3 for each sensor)
- Gyro bias and noise (stationary test)
- Accelerometer accuracy (gravity = 9.81 m/s²)

---

## Success Criteria

✅ Single command launches everything: `ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py`

✅ No manual timing or tmux required

✅ All components start cleanly with proper delays

✅ Hoverboard uses `/dev/ttyAMA0` (verified in logs)

✅ RPLidar uses `/dev/ttyUSB1` (verified in logs)

✅ Sensor fusion publishes `/odometry/filtered`

✅ IMU test shows good calibration and low noise

✅ Can drive robot with teleop

✅ SLAM creates maps successfully

---

**If everything works, you can delete:**
- `~/hoverbot/scripts/hoverbot_startup.sh` (tmux version)
- `~/hoverbot/ros2_ws/src/hoverbot_bringup/launch/hoverbot_full_v2.launch.py` (buggy version)

Congratulations! Your robot is now production-ready with proper sensor fusion! 🎉
