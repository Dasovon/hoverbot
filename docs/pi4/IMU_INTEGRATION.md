# BNO055 IMU Integration - Success!

## Date: December 28, 2025

## Hardware Setup ✅

**Sensor:** Adafruit BNO055 9-DOF Absolute Orientation IMU
**Connection:** I2C bus 1, address 0x28

**Wiring:**
- VIN → Pin 1 (3.3V)
- GND → Pin 6 (GND)
- SDA → Pin 3 (GPIO 2)
- SCL → Pin 5 (GPIO 3)

**I2C Configuration:**
```bash
# Verify I2C enabled
ls /dev/i2c-1  # Should exist

# Detect sensor
i2cdetect -y 1  # Shows 0x28
```

## Software Installation ✅

**Python Libraries:**
```bash
sudo pip3 install adafruit-circuitpython-bno055
sudo pip3 install RPi.GPIO
```

**ROS 2 Packages:**
```bash
sudo apt install ros-humble-bno055 ros-humble-imu-tools ros-humble-robot-localization
```

## Testing Results ✅

### Python Test
```bash
python3 test_bno055.py
```

**Output:**
- ✅ Temperature: 26-27°C
- ✅ Accelerometer: ~9.5 m/s² (gravity detection)
- ✅ Gyroscope: Small values (sensor stationary)
- ✅ Magnetometer: Detecting magnetic field
- ✅ Euler angles: Accurate orientation
- ✅ Quaternion: Stable output
- ✅ Calibration: Gyro auto-calibrated (3/3)

### ROS 2 Integration
```bash
ros2 run bno055 bno055 --ros-args \
  -p connection_type:=i2c \
  -p i2c_bus:=1 \
  -p i2c_addr:=0x28
```

**Topics Published:**
- `/bno055/imu` - Main IMU data (20 Hz) ✅
- `/bno055/calib_status` - Calibration status ✅
- `/bno055/mag` - Magnetometer data ✅
- `/bno055/grav` - Gravity vector ✅
- `/bno055/imu_raw` - Raw sensor data ✅
- `/bno055/temp` - Temperature ✅

**Data Quality:**
```bash
ros2 topic hz /bno055/imu
# Average rate: 20 Hz ✅
```

## Launch Files Created ✅

### Standalone IMU Launch
**File:** `ros2_ws/src/hoverbot_bringup/launch/imu.launch.py`

**Usage:**
```bash
ros2 launch hoverbot_bringup imu.launch.py
```

**Configuration:**
- I2C bus 1, address 0x28
- 20 Hz update rate
- NDOF mode (9DOF sensor fusion)
- Frame ID: `imu_link`

### Integrated with Robot Launch
**File:** `ros2_ws/src/hoverbot_bringup/launch/hoverbot_full_v2.launch.py`

**Component Sequence:**
1. Driver (0s)
2. TF Static (2s)
3. RPLidar (8s)
4. SLAM (10s)
5. **IMU (11s)** ← Added!

## Benefits for HoverBot 🚀

### Immediate Benefits
1. **Better Odometry** - Fuse IMU + wheel encoders for accuracy
2. **Improved SLAM** - More accurate heading/yaw
3. **Safety** - Detect if robot tips over
4. **Absolute Heading** - Magnetometer provides compass reference

### Future Capabilities
1. **robot_localization** - EKF fusion of odometry + IMU
2. **Collision Detection** - Monitor acceleration spikes
3. **Inclination Awareness** - Detect slopes/ramps
4. **Visual-Inertial SLAM** - Combine with camera later

## Known Issues & Solutions

### Issue: Combined Launch Still Uses Wrong Ports
**Status:** Under investigation

**Workaround:** Use tmux startup script (proven working)
```bash
cd ~/hoverbot/scripts
./hoverbot_startup.sh
```

**Individual launch files work perfectly:**
- `ros2 launch hoverbot_driver hoverbot_driver.launch.py` ✅
- `ros2 launch hoverbot_bringup imu.launch.py` ✅
- RPLidar via tmux timing ✅

## Next Steps

### For Next Session
1. Debug combined launch file port configuration
2. Test IMU data quality during robot movement
3. Set up `robot_localization` for odometry fusion
4. Calibrate IMU for robot mounting orientation

### Optional Enhancements
1. Save/load IMU calibration offsets
2. Create IMU health monitoring
3. Add IMU data to RViz visualization
4. Tune covariance values for sensor fusion

## Files Modified/Created

**New Files:**
- `ros2_ws/src/hoverbot_bringup/launch/imu.launch.py`
- `test_bno055.py` (Python test script on Pi)

**Modified Files:**
- `ros2_ws/src/hoverbot_bringup/launch/hoverbot_full_v2.launch.py`
- `ros2_ws/src/hoverbot_bringup/CMakeLists.txt`

**Configuration:**
- `/boot/firmware/config.txt` - GPU memory allocation

## References

**Hardware:**
- [Adafruit BNO055 Overview](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
- [BNO055 Datasheet](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bno055-absolute-orientation-sensor.pdf)

**ROS 2:**
- [ros-humble-bno055 Package](https://index.ros.org/p/bno055/)
- [robot_localization Documentation](http://docs.ros.org/en/humble/p/robot_localization/)

**Sensor Fusion:**
- IMU provides angular velocity and linear acceleration
- Fuses with wheel odometry for robust localization
- Essential for autonomous navigation

---

**Status:** ✅ IMU fully integrated and publishing at 20 Hz  
**Ready for:** Odometry fusion and autonomous navigation testing  
**Tested:** December 28, 2025