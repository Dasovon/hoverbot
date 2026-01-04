# HoverBot Driver - Installation Instructions

## Download Package

**Package**: `hoverbot_driver.tar.gz` (24 KB)

This archive contains the complete ROS 2 driver package with:
- Serial communication module (validated protocol)
- Differential drive kinematics
- Main driver node with 50Hz control loop
- Configuration files
- Launch files
- Test scripts
- Complete documentation

## Installation Steps

### 1. Download and Extract

```bash
# Download hoverbot_driver.tar.gz to ~/Downloads
# (Use the download link provided)

cd ~/Downloads
tar -xzf hoverbot_driver.tar.gz

# Verify extraction
ls hoverbot_driver/
# Should show: hoverbot_driver/ config/ launch/ test/ *.md *.xml *.py
```

### 2. Copy to Your Workspace

```bash
# Copy to your existing hoverbot repository
cp -r hoverbot_driver ~/hoverbot/ros2_ws/src/

# Verify
ls ~/hoverbot/ros2_ws/src/
# Should show: hoverbot_description  hoverbot_bringup  hoverbot_driver
```

### 3. Install Dependencies

```bash
cd ~/hoverbot/ros2_ws

# Install any missing ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Python serial library (if not already installed)
pip3 install pyserial
```

### 4. Build Package

```bash
cd ~/hoverbot/ros2_ws

# Build only the driver package
colcon build --packages-select hoverbot_driver

# Source the workspace
source install/setup.bash
```

Expected output:
```
Starting >>> hoverbot_driver
Finished <<< hoverbot_driver [0.5s]

Summary: 1 package finished [0.7s]
```

### 5. Verify Installation

```bash
# Check package is installed
ros2 pkg list | grep hoverbot
# Should show:
#   hoverbot_bringup
#   hoverbot_description
#   hoverbot_driver

# Check executable is available
ros2 pkg executables hoverbot_driver
# Should show: hoverbot_driver hoverbot_driver_node

# Check launch file
ros2 launch hoverbot_driver hoverbot_driver.launch.py --show-args
# Should show available launch arguments
```

### 6. Hardware Test (Important!)

```bash
# Set serial port permissions
sudo chmod 666 /dev/ttyAMA0

# Test serial communication BEFORE running ROS
cd ~/hoverbot/ros2_ws/src/hoverbot_driver/test
python3 test_serial_protocol.py
```

**Expected output:**
```
============================================================
HoverBot Serial Protocol Test
============================================================

Port: /dev/ttyAMA0
Baud: 115200

Attempting to connect...
✓ Serial port opened successfully

Sending test commands (zero velocity)...

✓ Valid feedback received!
  Battery: 36.50 V
  Temperature: 23.5 °C
  Speed Left: 0 RPM
  Speed Right: 0 RPM

============================================================
Test Results:
============================================================
  Commands sent:     150
  Feedback received: 148
  Errors:            0
  Success rate:      98.7%

✓ PASS - Communication working well!
  Your hoverboard is responding correctly.
  Ready to run ROS 2 driver node.
```

**If test fails, troubleshoot hardware before continuing.**

### 7. Launch Driver

```bash
cd ~/hoverbot/ros2_ws
source install/setup.bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

**Expected console output:**
```
[INFO] [launch]: All log files can be found below /home/ryan/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hoverbot_driver]: Connected to hoverboard on /dev/ttyAMA0 at 115200 baud
[INFO] [hoverbot_driver]: Robot limits:
[INFO] [hoverbot_driver]:   Max linear velocity: 0.26 m/s
[INFO] [hoverbot_driver]:   Max angular velocity: 1.29 rad/s
[INFO] [hoverbot_driver]:   Max wheel speed: 0.26 m/s (300 RPM)
[INFO] [hoverbot_driver]: HoverBot driver node started
```

**Hoverboard should NOT be beeping** (indicates commands being received).

### 8. Verify Topics

```bash
# Terminal 2 - check topics are publishing
ros2 topic list
# Should show:
#   /cmd_vel
#   /odom
#   /diagnostics
#   /tf

# Check odometry publishing rate
ros2 topic hz /odom
# Should show: average rate: 50.000

# Check diagnostics
ros2 topic echo /diagnostics --once
# Should show battery voltage, temperature
```

### 9. Test Motion

```bash
# Terminal 3 - install teleop if needed
sudo apt install ros-jazzy-teleop-twist-keyboard

# Run keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Use keyboard to control:**
- `i` = forward
- `,` = backward  
- `j` = rotate left
- `l` = rotate right
- `k` = stop

**Wheels should move immediately!**

### 10. Commit to Git

```bash
cd ~/hoverbot

# Check status
git status
# Should show: ros2_ws/src/hoverbot_driver/ (untracked)

# Add package
git add ros2_ws/src/hoverbot_driver/

# Commit
git commit -m "Add ROS 2 serial driver for hoverboard control

- Implements USART protocol for EFeru firmware
- Publishes odometry from wheel encoders  
- Supports differential drive kinematics
- Includes safety features and diagnostics
- Ready for Nav2 integration"

# Push to GitHub
git push origin main
```

## Verification Checklist

- [ ] Package extracted to `~/hoverbot/ros2_ws/src/hoverbot_driver/`
- [ ] Dependencies installed (`rosdep`, `pyserial`)
- [ ] Package builds without errors
- [ ] Test script passes (communication validated)
- [ ] Driver launches without errors
- [ ] No beeping from hoverboard
- [ ] `/odom` publishing at 50 Hz
- [ ] `/diagnostics` shows battery and temperature
- [ ] Wheels respond to teleop keyboard
- [ ] Robot moves in expected directions
- [ ] Changes committed to Git
- [ ] Pushed to GitHub

## Troubleshooting

### Build Fails
```bash
# Clean and rebuild
cd ~/hoverbot/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select hoverbot_driver
```

### Serial Port Access Denied
```bash
# Check permissions
ls -l /dev/ttyAMA0

# Fix permissions
sudo chmod 666 /dev/ttyAMA0

# Make permanent (survives reboot)
echo 'KERNEL=="ttyAMA0", MODE="0666"' | sudo tee /etc/udev/rules.d/99-serial.rules
sudo udevadm control --reload-rules
```

### Driver Won't Start
```bash
# Check serial port exists
ls /dev/ttyAMA*

# Check UART enabled
sudo raspi-config
# Interface Options → Serial Port
# Login shell: NO
# Serial hardware: YES
```

### Hoverboard Beeping
- Means firmware timeout (not receiving commands)
- Check driver is actually running: `ros2 node list`
- Check serial connection (TX↔RX, GND)
- Run test script to verify hardware

### Wheels Don't Move
```bash
# Check battery voltage
ros2 topic echo /diagnostics
# Battery should be >30V

# Check topics
ros2 topic list
# /cmd_vel should exist

# Publish test command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once
# Wheels should move slightly
```

## Next Steps

Once driver is working:

1. **Update bringup package** - Add driver to your launch files
2. **Tune parameters** - Adjust wheel diameter, wheelbase if needed
3. **Integrate SLAM** - Add to slam.launch.py
4. **Configure Nav2** - Add to navigation.launch.py
5. **Test autonomous navigation** - Set goals in RViz

## Documentation

- **README.md** - Complete package documentation
- **QUICKSTART.md** - Fast getting started guide  
- **INTEGRATION.md** - How to integrate with existing repo
- **MIGRATION.md** - Understanding Python → ROS transition
- **DELIVERY.md** - Package overview and specifications

## Support

**Hardware Issues:**
- Run `test_serial_protocol.py` to isolate problem
- Check wiring: Pi TX→Hoverboard RX, Pi RX→Hoverboard TX
- Verify firmware: VARIANT_USART, CONTROL_SERIAL_USART3, TANK_STEERING

**ROS Issues:**
- Check node is running: `ros2 node list`
- Verify topics exist: `ros2 topic list`
- Check parameters: `ros2 param list /hoverbot_driver`

**Integration Questions:**
- See INTEGRATION.md for adding to bringup
- See example launch files in `integration/`

## Summary

**Installation is complete when:**
- Package builds successfully
- Test script passes (PASS message)
- Driver launches without errors
- Hoverboard doesn't beep
- Wheels respond to keyboard teleop
- Changes pushed to GitHub

**You now have a working ROS 2 robot ready for autonomous navigation!**
