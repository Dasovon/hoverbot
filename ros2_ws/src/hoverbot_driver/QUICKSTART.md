# HoverBot Driver - Quick Start Guide

## System Requirements

**Supported Platforms:**
- **Development:** Ubuntu 22.04 + ROS 2 Humble (x86_64)
- **Deployment:** Ubuntu 24.04 + ROS 2 Jazzy (Raspberry Pi 5, ARM64)
- **Alternative:** Ubuntu 22.04 + ROS 2 Humble on Raspberry Pi 4/5

**Note:** Package works on both platforms - build separately for each architecture.

## Prerequisites Checklist

- [ ] Hoverboard firmware flashed with correct settings:
  - [ ] `VARIANT_USART` enabled
  - [ ] `CONTROL_SERIAL_USART3` enabled (right sideboard)
  - [ ] `TANK_STEERING` enabled
  - [ ] `BOARD_VARIANT = 1` (critical!)
  - [ ] Baud rate: 115200

- [ ] Hardware connected:
  - [ ] Pi GPIO14 (TX) → Hoverboard RX (green wire)
  - [ ] Pi GPIO15 (RX) → Hoverboard TX (blue wire)
  - [ ] Pi GND → Hoverboard GND (black wire)
  - [ ] **NOT CONNECTED**: Red wire (15V!)

- [ ] UART enabled on Raspberry Pi:
  - [ ] `sudo raspi-config` → Interface Options → Serial Port
  - [ ] Login shell: **NO**, Serial hardware: **YES**

- [ ] Battery connected and charged (>30V)

## Installation (5 minutes)

**Choose your platform:**
- **Development Machine (Ubuntu 22.04 + Humble):** See `INSTALL_DEV.md`
- **Raspberry Pi (Ubuntu 24.04 + Jazzy):** Continue below

### On Raspberry Pi:

```bash
# 1. Copy package to your workspace
cd ~/ros2_ws/src
cp -r /path/to/hoverbot_driver .

# 2. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --packages-select hoverbot_driver
source install/setup.bash

# 4. Set serial port permissions (required after each reboot)
sudo chmod 666 /dev/ttyAMA0
```

## Testing (Before Running ROS)

```bash
# Test serial communication first (important!)
cd ~/ros2_ws/src/hoverbot_driver/test
python3 test_serial_protocol.py

# Expected output:
#   ✓ Serial port opened successfully
#   ✓ Valid feedback received!
#   Battery: 36.50 V
#   Temperature: 23.5 °C
#   ✓ PASS - Communication working well!
```

**If test fails**, fix hardware/firmware before proceeding to ROS.

## Running the Driver

### Terminal 1: Launch Driver
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

**Expected console output:**
```
[INFO] [hoverbot_driver]: Connected to hoverboard on /dev/ttyAMA0 at 115200 baud
[INFO] [hoverbot_driver]: Robot limits:
[INFO] [hoverbot_driver]:   Max linear velocity: 0.26 m/s
[INFO] [hoverbot_driver]:   Max angular velocity: 1.29 rad/s
[INFO] [hoverbot_driver]: HoverBot driver node started
```

### Terminal 2: Check Topics
```bash
# Verify odometry publishing
ros2 topic hz /odom
# Should show ~50 Hz

# Verify diagnostics
ros2 topic echo /diagnostics
# Should show battery voltage, temperature
```

### Terminal 3: Test Motion
```bash
# Install teleop if needed
sudo apt install ros-jazzy-teleop-twist-keyboard

# Run keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel

# Use arrow keys to drive
# i/k = forward/backward
# j/l = rotate left/right
```

**Wheels should move immediately!**

## Verification Checklist

### ✅ Driver Running Successfully When:
- [ ] No beeping from hoverboard
- [ ] Console shows "HoverBot driver node started"
- [ ] `/odom` topic publishing at 50 Hz
- [ ] `/diagnostics` shows battery voltage
- [ ] Wheels respond to keyboard teleop
- [ ] Robot moves in expected direction
- [ ] Robot stops when teleop released

### ⚠️ Common Issues

**Continuous beeping from hoverboard:**
- Cause: Firmware timeout (not receiving commands)
- Fix: Ensure driver node is running, check serial connection

**Wheels don't move:**
- Check: Serial permissions (`ls -l /dev/ttyAMA0`)
- Check: Battery voltage >30V
- Check: Correct UART pins connected
- Check: Driver node actually running (check `ros2 node list`)

**Wrong movement direction:**
- Expected with tank steering - will fix with proper parameters
- Swap left/right commands in config if needed

**No `/odom` topic:**
- Check: `publish_odom: true` in config file
- Check: Driver node actually started (no errors in console)

## Next Steps

Once basic driving works:

1. **Tune odometry** - Measure actual distances and adjust wheel parameters
2. **Add SLAM** - Integrate `slam_toolbox` for mapping
3. **Configure Nav2** - Setup autonomous navigation
4. **Add sensors** - Integrate RPLidar A1 for obstacle avoidance

## Stopping the Robot

**Emergency stop:**
```bash
# Ctrl+C in teleop window
# OR
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}" --once
```

**Clean shutdown:**
```bash
# Stop driver node (Ctrl+C in driver window)
# Driver sends zero command before disconnecting
```

## Parameter Tuning

Edit `config/hoverbot_driver.yaml`:

```yaml
# Increase speed limit (careful!)
max_rpm: 400  # Default: 300

# Adjust for your specific wheels
wheel_diameter: 0.165  # Measure your wheels
wheelbase: 0.40  # Measure your track width

# Increase safety timeout
cmd_vel_timeout: 1.0  # Default: 0.5 seconds
```

After editing, rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select hoverbot_driver
source install/setup.bash
```

## Performance Monitoring

```bash
# Check diagnostics in real-time
ros2 topic echo /diagnostics

# Monitor odometry
ros2 topic echo /odom

# View TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf showing transform tree

# Check node status
ros2 node info /hoverbot_driver
```

## Troubleshooting Commands

```bash
# List available serial ports
ls -l /dev/ttyAMA*

# Check UART configuration
sudo raspi-config nonint get_serial
# Should output: 0 1 (login disabled, hardware enabled)

# Monitor serial traffic (debugging)
sudo apt install minicom
minicom -D /dev/ttyAMA0 -b 115200

# Check ROS 2 environment
printenv | grep ROS
```

## Getting Help

1. **Hardware issues**: Check wiring, battery, firmware configuration
2. **Driver issues**: Check logs in terminal, verify parameters
3. **ROS issues**: Check topic list (`ros2 topic list`), node list (`ros2 node list`)

## Success Criteria

You're ready to move forward when:
- Driver starts without errors
- Hoverboard doesn't beep continuously
- Teleop keyboard controls the robot smoothly
- `/odom` updates when robot moves
- Battery voltage and temperature visible in diagnostics

**That's it! You now have a working ROS 2 robot base.**
