# HoverBot Project - Session Scratchpad

**Last Updated:** 2026-01-04  
**Current Status:** Hoverboard driver fully operational, BNO055 IMU next

---

## ✅ What's Working

### Hoverboard Driver (COMPLETE)
- **Motor Control**: Both wheels respond to cmd_vel commands
- **Telemetry**: Receiving battery (39.2V), temperature (33°C), wheel RPMs
- **Odometry**: Publishing at 50Hz on `/odom`
- **TF**: Broadcasting `odom → base_link` at 50Hz
- **Diagnostics**: Publishing at 5Hz on `/diagnostics`
- **RViz Visualization**: Forward movement displays correctly

**Critical Fix Applied:**
- Frame synchronization: Byte-by-byte scan for start frame `0xCD 0xAB`
- Right wheel sign correction: Firmware reports right wheel RPM as negative, negated in odometry calculation

### RPLidar A1 (COMPLETE)
- **Device**: `/dev/ttyUSB1`
- **Publishing**: `/scan` topic active
- **Permissions**: Udev rule created for persistent access
- **Network**: Visible from dev machine (ROS_DOMAIN_ID=42)

### Network Configuration (COMPLETE)
- **ROS_DOMAIN_ID**: 42 on both machines
- **ROS_LOCALHOST_ONLY**: 0 on both machines
- **Communication**: Dev machine can see Pi topics/TF

### GitHub Repository (SYNCED)
- **URL**: github.com/Dasovon/hoverbot
- **Branch**: main
- **SSH Keys**: Configured on both dev and Pi
- **Status**: Both machines synced to latest commit

---

## 🔧 In Progress / Broken

### BNO055 IMU (BROKEN)
- **Hardware**: Detected at I2C address 0x28 (40 decimal) on `/dev/i2c-1`
- **Driver**: `flynneva/bno055` package installed and built
- **Issue**: Node configures successfully but freezes - no topics published
- **Launch Command**: `ros2 run bno055 bno055 --ros-args -p connection_type:=i2c -p i2c_bus:=1 -p i2c_addr:=40`
- **Next Steps**: Try alternative driver or custom Python implementation

### Cameras (NOT STARTED)
- RealSense D435 - not configured
- ELP 2MP Camera - not configured

### Robot Description (REMOVED)
- `hoverbot_description` package removed during cleanup
- `hoverbot_bringup` package removed during cleanup
- **Impact**: No URDF/robot model in RViz (not critical for current testing)

---

## 📁 System Configuration

### Pi (hostname: hoverbot)
**Hardware:**
- Raspberry Pi 4 (8GB)
- RPLidar A1 on `/dev/ttyUSB1`
- BNO055 IMU on I2C bus 1, address 0x28
- Hoverboard on `/dev/ttyAMA0` (UART)

**Software:**
- Ubuntu 22.04 Jammy
- ROS2 Humble (base install via apt)
- Workspace: `~/hoverbot/ros2_ws/`

**Packages:**
- `hoverbot_driver` - Custom hoverboard driver
- `rplidar_ros` - SLAMTEC RPLidar driver
- `bno055` - IMU driver (broken)

**Key Files:**
- `~/.bashrc`: Sources ROS2, sets ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=0
- `/boot/firmware/config.txt`: UART enabled (`enable_uart=1`, `dtoverlay=disable-bt`)

### Dev Machine (hostname: dev)
**Hardware:**
- x86_64 desktop

**Software:**
- Ubuntu 22.04 Jammy
- ROS2 Humble (binary install)
- Workspace: `~/hoverbot/ros2_ws/`

**Key Files:**
- `~/.bashrc`: Sources `~/ros2_humble/ros2-linux/setup.bash`, ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=0

---

## 🐛 Known Issues & Workarounds

### Issue 1: Right Wheel RPM Sign
**Problem:** Hoverboard firmware reports right wheel RPM as negative even when spinning forward  
**Workaround:** Negate right wheel RPM in `hoverbot_driver_node.py` line 203  
**Code:** `self.update_odometry(feedback.speed_l_rpm, -feedback.speed_r_rpm)`

### Issue 2: Odometry Drift
**Problem:** Robot slowly veers left (~1-2 RPM difference between wheels)  
**Status:** Normal for dead reckoning, will be corrected by SLAM  
**RPM Difference:** Left: 76-78 RPM, Right: -76 to -79 RPM (1-3 RPM variation)

### Issue 3: Telemetry Frame Synchronization
**Problem:** Original driver couldn't sync to packet boundaries  
**Solution:** Implemented byte-by-byte start frame scanning in `serial_interface.py`  
**Details:** Scans for `0xCD 0xAB` (little-endian 0xABCD), validates checksum

---

## 📝 Recent Changes (2026-01-04)

1. **Fixed telemetry reception**
   - Added frame synchronization with byte-by-byte scanning
   - Implemented sync buffer to handle packet boundaries
   - File: `serial_interface.py`

2. **Fixed odometry calculation**
   - Negated right wheel RPM (firmware reports backwards)
   - File: `hoverbot_driver_node.py` line 203

3. **Cleaned up debug code**
   - Removed debug print statements
   - Cleaned up comments
   - Final versions deployed and tested

4. **GitHub sync**
   - Both dev and Pi synced to main branch
   - All working code committed

---

## 🎯 Next Steps (Priority Order)

1. **Fix BNO055 IMU** (CURRENT)
   - Try alternative driver packages
   - Consider custom Python implementation using Adafruit library
   - Expected topics: `/imu/data`, `/imu/mag`, `/imu/temp`

2. **Add Robot Description**
   - Create minimal URDF for RViz visualization
   - Define robot dimensions, wheel positions
   - Package: `hoverbot_description`

3. **Camera Integration**
   - RealSense D435 setup
   - ELP 2MP camera setup
   - Test depth/RGB streaming

4. **SLAM Setup**
   - Install `slam_toolbox`
   - Create maps of environment
   - Tune SLAM parameters

5. **Navigation Stack**
   - Install Nav2
   - Configure costmaps
   - Test autonomous navigation

---

## 🔑 Important Commands

### Start Hoverboard Driver
```bash
# On Pi
source ~/hoverbot/ros2_ws/install/setup.bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

### Test Movement
```bash
# Send forward command
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### Start RPLidar
```bash
# On Pi
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
```

### Check Topics
```bash
ros2 topic list
ros2 topic hz /odom
ros2 topic echo /diagnostics
```

### RViz on Dev
```bash
source ~/hoverbot/ros2_ws/install/setup.bash
rviz2
# Set Fixed Frame: odom
# Add: TF, Odometry (/odom)
```

### GitHub Sync
```bash
# On Pi or Dev
cd ~/hoverbot/ros2_ws
git pull origin main
git add -A
git commit -m "Description of changes"
git push origin main
```

---

## 📊 Performance Metrics

### Hoverboard Driver
- Control loop: 50Hz (20ms period)
- Odometry publishing: 50Hz
- TF broadcasting: 50Hz
- Diagnostics: 5Hz
- Telemetry reception rate: ~100Hz from firmware

### Communication
- TX success rate: 100%
- RX success rate: 100%
- Checksum errors: 0
- Framing errors: 0

---

## 🔬 Hardware Details

### Hoverboard Configuration
- **Firmware**: EFeru hoverboard-firmware-hack-FOC
- **Variant**: BOARD_VARIANT=1
- **Control Mode**: TANK_STEERING, CONTROL_SERIAL_USART3
- **Feedback**: FEEDBACK_SERIAL_USART3 enabled
- **Baud Rate**: 115200
- **Protocol**: Custom binary (8-byte command, 18-byte feedback)
- **Wheel Diameter**: 0.165m
- **Wheelbase**: 0.40m
- **Max RPM**: 300

### RPLidar A1
- **Connection**: USB serial
- **Device**: `/dev/ttyUSB1`
- **Scan Rate**: 5.5 Hz
- **Range**: 0.15m - 12m

### BNO055 IMU
- **Connection**: I2C bus 1
- **Address**: 0x28 (7-bit) / 40 (decimal)
- **Sensor**: 9-DOF (accel, gyro, mag)
- **Fusion Mode**: NDOF (Nine Degrees of Freedom)

---

## 📖 Reference Documentation

### Local Files
- `/mnt/skills/public/` - Claude skills for document creation
- Old working code: Backed up in uploaded files

### External Resources
- EFeru Firmware: https://github.com/EFeru/hoverboard-firmware-hack-FOC
- RPLidar ROS2: https://github.com/Slamtec/rplidar_ros
- BNO055 ROS2: https://github.com/flynneva/bno055
- ROS2 Humble Docs: https://docs.ros.org/en/humble/

---

## 💾 Backup & Recovery

### Critical Files to Preserve
1. `~/hoverbot/ros2_ws/src/hoverbot_driver/` - Custom driver code
2. `~/.bashrc` - ROS environment setup
3. `/boot/firmware/config.txt` - UART/I2C configuration
4. SSH keys: `~/.ssh/id_ed25519` (both machines)

### Quick Recovery Steps
```bash
# If driver breaks, restore from GitHub
cd ~/hoverbot/ros2_ws/src
rm -rf hoverbot_driver
git clone github.com:Dasovon/hoverbot.git temp
mv temp/hoverbot_driver .
rm -rf temp
colcon build --packages-select hoverbot_driver
```

---

## ⚠️ Important Notes

1. **Always source workspace** before running ROS commands
2. **Right wheel RPM is negative** in firmware - this is EXPECTED
3. **Frame sync is critical** - don't remove the byte-by-byte scanning
4. **Diagnostics show raw RPM** (right wheel negative), odometry uses corrected values
5. **Minor drift is normal** - 1-2 RPM difference causes slow veering
6. **Python print() is buffered** - may not see debug output until Ctrl+C
7. **ROS_DOMAIN_ID must match** on all machines for communication
8. **GitHub has latest working code** - pull before making changes

---

## 🎓 Lessons Learned

1. **Telemetry sync requires frame alignment** - can't just read 18-byte chunks
2. **Firmware quirks exist** - right wheel RPM sign inversion
3. **Test incrementally** - raw serial test revealed telemetry was flowing
4. **Old working code is gold** - comparison showed exact same code worked before
5. **Network issues look like code issues** - always check ROS_DOMAIN_ID first
6. **Hardware verification first** - `i2cdetect`, `cat /dev/ttyAMA0 | hexdump`

---

**END OF SCRATCHPAD**

*This file should be updated at the end of each session with new progress, issues, and next steps.*
