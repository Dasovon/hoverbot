# HoverBot Quick Start Guide

Get your HoverBot up and running in minutes with this single-page guide.

## Prerequisites

### Hardware Checklist

- [ ] Raspberry Pi 4 (4GB+ recommended) running Ubuntu 22.04
- [ ] Hoverboard with EFeru firmware (BOARD_VARIANT=1, TANK_STEERING)
- [ ] RPLidar A1 sensor
- [ ] BNO055 IMU sensor
- [ ] UART connection: Pi GPIO → Hoverboard USART3
- [ ] Hoverboard powered on (battery charged)
- [ ] Dev machine on same network as Pi

### Required Connections

| Component | Connection | Device Path |
|-----------|-----------|-------------|
| Hoverboard | Pi GPIO UART (pins 8/10) | `/dev/ttyAMA0` |
| RPLidar A1 | Pi USB port | `/dev/ttyUSB1` (or `/dev/ttyUSB0`) |
| BNO055 IMU | Pi I2C (pins 3/5) | I2C bus 1, address 0x28 |

## Software Installation

### On Raspberry Pi

```bash
# 1. Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-ros-base python3-colcon-common-extensions

# 2. Clone the repository
cd ~
git clone github.com:Dasovon/hoverbot.git ros2_ws
cd ros2_ws

# 3. Install dependencies
sudo apt install ros-humble-rplidar-ros
sudo pip3 install adafruit-circuitpython-bno055

# 4. Build the workspace
colcon build --symlink-install
source install/setup.bash

# 5. Configure ROS2 networking
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
source ~/.bashrc
```

### On Dev Machine

```bash
# 1. Install ROS2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# 2. Configure ROS2 networking to match Pi
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
source ~/.bashrc

# 3. (Optional) Clone repo for development
cd ~
git clone github.com:Dasovon/hoverbot.git ros2_ws
cd ros2_ws
colcon build
source install/setup.bash
```

## Hardware Configuration

### Enable UART (Pi)

```bash
# Edit boot configuration
sudo nano /boot/firmware/config.txt

# Add these lines:
enable_uart=1
dtoverlay=disable-bt

# Reboot
sudo reboot
```

### Enable I2C (Pi)

```bash
# Enable I2C interface
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable

# Add user to i2c group
sudo usermod -a -G i2c,gpio $USER

# Reboot to apply
sudo reboot
```

### Configure RPLidar Permissions (Pi)

```bash
# Create udev rule for persistent USB access
sudo nano /etc/udev/rules.d/rplidar.rules

# Add this line:
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group
sudo usermod -a -G dialout $USER
```

## Verify Hardware

Before starting ROS2 nodes, verify all hardware is detected:

```bash
# Hoverboard UART (should show data flowing)
sudo cat /dev/ttyAMA0 | hexdump -C
# Press Ctrl+C after seeing data

# RPLidar USB
ls -l /dev/ttyUSB*
# Should show: /dev/ttyUSB0 or /dev/ttyUSB1

# BNO055 IMU (should show address 28)
sudo i2cdetect -y 1
```

## Launch All Sensors

### On Raspberry Pi

Open **three separate terminal sessions** on the Pi (via SSH or directly):

#### Terminal 1: Hoverboard Driver

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

**Expected output:**
```
[INFO] Connected to hoverboard on /dev/ttyAMA0 at 115200 baud
[INFO] Robot limits:
[INFO]   Max linear velocity: 1.57 m/s
[INFO]   Max angular velocity: 3.93 rad/s
[INFO] HoverBot driver node started
```

#### Terminal 2: RPLidar

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
```

**Note:** If `/dev/ttyUSB1` doesn't work, try `/dev/ttyUSB0`

**Expected output:**
```
[INFO] RPLIDAR running on ROS2 package rplidar_ros
[INFO] SDK Version: 2.0
[INFO] Firmware Version: 1.xx
```

#### Terminal 3: BNO055 IMU

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch bno055_driver bno055.launch.py
```

**Expected output:**
```
[INFO] BNO055 sensor initialized successfully
[INFO] BNO055 node started at 50.0 Hz
[INFO] Publishing on topics: /imu/data, /imu/mag, /imu/temp
```

## Verify Topics Are Publishing

On the Pi or dev machine:

```bash
# List all active topics
ros2 topic list

# Expected topics:
# /cmd_vel
# /diagnostics
# /imu/data
# /imu/mag
# /imu/temp
# /odom
# /robot_active
# /scan
# /tf

# Check publishing rates
ros2 topic hz /odom        # Should show ~50 Hz
ros2 topic hz /scan        # Should show ~5.5 Hz
ros2 topic hz /imu/data    # Should show ~50 Hz

# Check data content
ros2 topic echo /diagnostics --once
ros2 topic echo /odom --once
ros2 topic echo /scan --once
```

## Visualize in RViz2

### On Dev Machine

```bash
# Start RViz2
rviz2
```

### Configure RViz2

1. **Set Fixed Frame:**
   - In left panel, change "Fixed Frame" from `map` to `odom`

2. **Add TF Display:**
   - Click "Add" button
   - Select "TF"
   - Click "OK"
   - You should see the `odom → base_link` transform

3. **Add Odometry Display:**
   - Click "Add"
   - Select "Odometry"
   - Under "Topic", select `/odom`
   - Expand "Shape" and set:
     - Keep arrows: 20
     - Arrow length: 0.3

4. **Add LaserScan Display:**
   - Click "Add"
   - Select "LaserScan"
   - Under "Topic", select `/scan`
   - Set "Size (m)": 0.05
   - Set "Color": red or green

5. **Save Configuration:**
   - File → Save Config As → `~/hoverbot_rviz.rviz`
   - Next time: `rviz2 -d ~/hoverbot_rviz.rviz`

## Test Robot Movement

### Safety First!

- Put robot on blocks or have emergency stop ready
- Ensure area is clear
- Be ready to Ctrl+C the hoverboard driver

### Send Test Commands

#### Method 1: Command Line (Recommended for testing)

```bash
# Move forward slowly (0.2 m/s)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Stop (Ctrl+C to stop publishing, then send zero command)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Rotate in place (0.5 rad/s)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Move backward
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist "{linear: {x: -0.2}, angular: {z: 0.0}}"
```

#### Method 2: Teleop Keyboard (More convenient)

```bash
# Install teleop_twist_keyboard
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i`: Forward
- `k`: Stop
- `,`: Backward
- `j`: Rotate left
- `l`: Rotate right
- `u`: Forward + rotate left
- `o`: Forward + rotate right
- `q`: Increase speed
- `z`: Decrease speed
- `Ctrl+C`: Quit

### Verify Odometry

While moving forward, watch RViz2:

- The `base_link` frame should move forward in RViz
- The odometry arrows should point in the direction of motion
- The `/odom` topic should show increasing `x` position

```bash
# Watch odometry live
ros2 topic echo /odom
```

### Check Diagnostics

```bash
# View battery, temperature, wheel speeds
ros2 topic echo /diagnostics
```

**Important values:**
- **Battery Voltage:** Should be 30V - 42V (10S lithium battery)
- **Temperature:** Should be < 60°C
- **Speed Left/Right:** Should show RPM when moving (right wheel is negative)
- **Checksum/Framing Errors:** Should be 0

## Common Issues

### "Failed to connect to serial port"

**Problem:** Hoverboard driver can't open `/dev/ttyAMA0`

**Solutions:**
- Verify UART is enabled: `ls /dev/ttyAMA0`
- Check `/boot/firmware/config.txt` has `enable_uart=1`
- Reboot Pi
- Check wiring: GPIO 14 (TX) and GPIO 15 (RX)

### "No topics visible from dev machine"

**Problem:** Can't see Pi topics on dev machine

**Solutions:**
- Verify ROS_DOMAIN_ID matches: `echo $ROS_DOMAIN_ID` (should be 42 on both)
- Verify ROS_LOCALHOST_ONLY=0 on both machines
- Check network connectivity: `ping <pi_hostname>`
- Check firewall isn't blocking multicast
- Restart ROS2 nodes

### "RPLidar device not found"

**Problem:** RPLidar not detected at `/dev/ttyUSB1`

**Solutions:**
- Check USB connection
- Try `/dev/ttyUSB0` instead
- Check permissions: `ls -l /dev/ttyUSB*`
- Verify udev rule is loaded
- Try: `sudo chmod 666 /dev/ttyUSB0` (temporary fix)

### "BNO055 sensor not detected"

**Problem:** IMU node fails with "Failed to initialize BNO055"

**Solutions:**
- Verify I2C enabled: `ls /dev/i2c-1`
- Check I2C address: `sudo i2cdetect -y 1` (should show `28`)
- Check wiring: SDA→GPIO2, SCL→GPIO3, 3.3V, GND
- Add user to groups: `sudo usermod -a -G i2c,gpio $USER` then reboot
- Install library: `sudo pip3 install adafruit-circuitpython-bno055`

### "cmd_vel timeout - stopping robot"

**Problem:** Robot stops after 0.5 seconds

**Solution:** This is normal safety behavior! Keep publishing cmd_vel at ≥2 Hz
- With `ros2 topic pub`, use `--rate 10`
- With teleop_twist_keyboard, keep holding keys
- For autonomous control, ensure your controller publishes continuously

### "Odometry drifts or moves backward"

**Problem:** Robot moves forward but odometry shows backward movement

**Solution:** This was already fixed! If you still see this:
- Verify you're running latest code from repository
- Check line 196 in `hoverbot_driver_node.py`: should have `-feedback.speed_r_rpm`
- The right wheel negation is CRITICAL - don't remove it!

## What's Next?

Now that your HoverBot is operational, you can:

1. **Create Robot Description**
   - Build URDF model for proper visualization
   - Define sensor frames and transforms

2. **Set Up SLAM**
   - Install `slam_toolbox`
   - Create maps of your environment
   - Enable autonomous navigation

3. **Add Cameras**
   - RealSense D435 for depth perception
   - ELP 2MP camera for vision

4. **Install Nav2**
   - Autonomous path planning
   - Obstacle avoidance
   - Goal-based navigation

See `SESSION_SCRATCHPAD.md` for detailed next steps and project status.

## Reference Documentation

- **Project Status:** `SESSION_SCRATCHPAD.md` - Current state, known issues, next steps
- **Hoverboard Driver:** `src/hoverbot_driver/README.md` - Driver API and details
- **BNO055 IMU:** `src/bno055_driver/README.md` - IMU setup and calibration
- **Code Analysis:** `docs/analysis/` - Architecture and refactoring proposals

## Getting Help

If you encounter issues not covered here:

1. Check `SESSION_SCRATCHPAD.md` for known issues
2. Check diagnostics: `ros2 topic echo /diagnostics`
3. Check node logs for error messages
4. Verify hardware with direct testing (before running ROS2 nodes)

## Command Reference Sheet

```bash
# Start all sensors (3 terminals on Pi)
ros2 launch hoverbot_driver hoverbot_driver.launch.py
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
ros2 launch bno055_driver bno055.launch.py

# Test movement
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Stop robot
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Check topics
ros2 topic list
ros2 topic hz /odom
ros2 topic echo /diagnostics

# Visualize
rviz2

# Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

**You're ready to roll! 🚀**
