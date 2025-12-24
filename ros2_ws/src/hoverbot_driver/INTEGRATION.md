# Integration Guide - Adding Driver to Your Existing Repository

## Your Current Repository Structure

```
hoverbot/
├── firmware/               # EFeru firmware configs
├── raspberry-pi/           # Python serial scripts (working!)
├── ros2_ws/
│   └── src/
│       ├── hoverbot_description/  # URDF files
│       └── hoverbot_bringup/      # Launch files
├── hardware/               # Wiring diagrams, docs
└── docs/                   # Documentation
```

## After Integration

```
hoverbot/
├── firmware/
├── raspberry-pi/
├── ros2_ws/
│   └── src/
│       ├── hoverbot_description/
│       ├── hoverbot_bringup/
│       └── hoverbot_driver/       # ← NEW: This package
├── hardware/
└── docs/
```

## Step-by-Step Integration

### 1. Copy Driver Package

```bash
# You should have downloaded hoverbot_driver.tar.gz
cd ~/Downloads
tar -xzf hoverbot_driver.tar.gz

# Copy to your workspace
cp -r hoverbot_driver ~/hoverbot/ros2_ws/src/

# Verify
ls ~/hoverbot/ros2_ws/src/
# Should show: hoverbot_description  hoverbot_bringup  hoverbot_driver
```

### 2. Build the Package

```bash
cd ~/hoverbot/ros2_ws
colcon build --packages-select hoverbot_driver
source install/setup.bash
```

Expected output:
```
Starting >>> hoverbot_driver
Finished <<< hoverbot_driver [0.50s]

Summary: 1 package finished
```

### 3. Test Hardware Connection

```bash
# Set serial permissions
sudo chmod 666 /dev/ttyAMA0

# Run standalone test
cd ~/hoverbot/ros2_ws/src/hoverbot_driver/test
python3 test_serial_protocol.py

# Expected:
# ✓ Serial port opened successfully
# ✓ Valid feedback received!
# ✓ PASS - Communication working well!
```

If test fails, fix hardware before continuing.

### 4. Launch Driver Standalone

```bash
cd ~/hoverbot/ros2_ws
source install/setup.bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

Expected console output:
```
[INFO] [hoverbot_driver]: Connected to hoverboard on /dev/ttyAMA0 at 115200 baud
[INFO] [hoverbot_driver]: Robot limits:
[INFO] [hoverbot_driver]:   Max linear velocity: 0.26 m/s
[INFO] [hoverbot_driver]:   Max angular velocity: 1.29 rad/s
[INFO] [hoverbot_driver]: HoverBot driver node started
```

### 5. Test Motion

```bash
# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Wheels should respond to keyboard commands!

### 6. Update hoverbot_bringup (Optional)

Add integrated launch file to your bringup package:

```bash
# Copy integration launch file
cp ~/hoverbot/ros2_ws/src/hoverbot_driver/integration/robot_bringup.launch.py \
   ~/hoverbot/ros2_ws/src/hoverbot_bringup/launch/

# Rebuild bringup
cd ~/hoverbot/ros2_ws
colcon build --packages-select hoverbot_bringup
source install/setup.bash

# Now you can launch everything at once:
ros2 launch hoverbot_bringup robot_bringup.launch.py
```

This launches:
- hoverbot_driver (hardware control)
- robot_state_publisher (URDF → TF)

### 7. Commit to Git

```bash
cd ~/hoverbot

# Check what's new
git status
# Should show: ros2_ws/src/hoverbot_driver/ (untracked)

# Add the driver
git add ros2_ws/src/hoverbot_driver/

# Commit
git commit -m "Add ROS 2 serial driver for hoverboard control

- Implements USART protocol for EFeru firmware
- Publishes odometry from wheel encoders
- Supports differential drive kinematics
- Includes safety features and diagnostics
- Integrates with Nav2 navigation stack"

# Push to GitHub
git push origin main
```

## Updating Existing Launch Files

### Your Current slam.launch.py

Probably looks like:
```python
# slam.launch.py (current)
def generate_launch_description():
    return LaunchDescription([
        # Robot state publisher
        robot_state_launch,
        
        # RPLidar
        lidar_node,
        
        # SLAM Toolbox
        slam_node
    ])
```

**Update to include driver:**
```python
# slam.launch.py (updated)
def generate_launch_description():
    return LaunchDescription([
        # Hardware driver (NEW!)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('hoverbot_driver'),
                    'launch',
                    'hoverbot_driver.launch.py'
                ])
            ])
        ),
        
        # Robot state publisher
        robot_state_launch,
        
        # RPLidar
        lidar_node,
        
        # SLAM Toolbox
        slam_node
    ])
```

### Your Current navigation.launch.py

**Update to include driver:**
```python
# navigation.launch.py
def generate_launch_description():
    return LaunchDescription([
        # Hardware driver (NEW!)
        driver_launch,
        
        # Robot state publisher
        robot_state_launch,
        
        # Nav2
        nav2_launch,
        
        # RViz
        rviz_node
    ])
```

## File Modifications Summary

### Files to ADD:
- `ros2_ws/src/hoverbot_driver/` (entire package)

### Files to MODIFY (optional):
- `ros2_ws/src/hoverbot_bringup/launch/slam.launch.py` (add driver)
- `ros2_ws/src/hoverbot_bringup/launch/navigation.launch.py` (add driver)

### Files UNCHANGED:
- `ros2_ws/src/hoverbot_description/` (URDF stays the same)
- `firmware/` (your working config stays the same)
- `raspberry-pi/` (Python scripts still work for testing)
- `hardware/` (wiring unchanged)

## Workspace Dependencies

Update your workspace dependencies:

```bash
cd ~/hoverbot/ros2_ws

# Install any missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild everything
colcon build

# Source
source install/setup.bash
```

## Testing Complete Integration

### Test 1: Driver Only
```bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
# Check: /odom publishing, wheels respond to /cmd_vel
```

### Test 2: Driver + State Publisher
```bash
ros2 launch hoverbot_bringup robot_bringup.launch.py
# Check: TF tree complete (odom→base_link→laser_frame)
```

### Test 3: Full SLAM
```bash
ros2 launch hoverbot_bringup slam.launch.py
# Drive around, watch map build in RViz
```

### Test 4: Autonomous Navigation
```bash
ros2 launch hoverbot_bringup navigation.launch.py
# Set nav goal in RViz, watch robot navigate
```

## Troubleshooting Integration

### Driver package not found
```bash
# Make sure it's in src/
ls ~/hoverbot/ros2_ws/src/hoverbot_driver
# Rebuild
cd ~/hoverbot/ros2_ws
colcon build --packages-select hoverbot_driver
source install/setup.bash
```

### Serial port permission denied
```bash
# Every boot, run:
sudo chmod 666 /dev/ttyAMA0
# Or add to startup:
echo 'KERNEL=="ttyAMA0", MODE="0666"' | sudo tee /etc/udev/rules.d/99-serial.rules
sudo udevadm control --reload-rules
```

### Wheels don't move
1. Test standalone Python script first (proves hardware works)
2. Check driver node is actually running (`ros2 node list`)
3. Verify /cmd_vel topic exists (`ros2 topic list`)
4. Check battery voltage in diagnostics

### Odometry drift
- Normal for dead reckoning
- Tune `wheel_diameter` and `wheelbase` in config
- SLAM corrects drift with laser scans

## Git Workflow

```bash
# Check status
cd ~/hoverbot
git status

# Stage driver
git add ros2_ws/src/hoverbot_driver/

# Stage updated launch files (if you modified bringup)
git add ros2_ws/src/hoverbot_bringup/launch/

# Commit
git commit -m "Integrate ROS 2 driver with navigation stack"

# Push
git push
```

## Directory Structure After Integration

```
hoverbot/
├── firmware/
│   └── config.h (VARIANT_USART, BOARD_VARIANT=1)
│
├── raspberry-pi/
│   ├── uart_setup/
│   │   └── test_serial.py (still works for debugging!)
│   └── python_control/
│
├── ros2_ws/
│   ├── src/
│   │   ├── hoverbot_description/
│   │   │   ├── urdf/
│   │   │   │   └── hoverbot.urdf.xacro
│   │   │   └── launch/
│   │   │       └── robot_state_publisher.launch.py
│   │   │
│   │   ├── hoverbot_bringup/
│   │   │   ├── config/
│   │   │   └── launch/
│   │   │       ├── robot_bringup.launch.py (NEW - integrated)
│   │   │       ├── slam.launch.py (updated with driver)
│   │   │       └── navigation.launch.py (updated with driver)
│   │   │
│   │   └── hoverbot_driver/           # ← NEW PACKAGE
│   │       ├── hoverbot_driver/
│   │       │   ├── __init__.py
│   │       │   ├── serial_interface.py
│   │       │   ├── differential_drive_controller.py
│   │       │   └── hoverbot_driver_node.py
│   │       ├── config/
│   │       │   └── hoverbot_driver.yaml
│   │       ├── launch/
│   │       │   └── hoverbot_driver.launch.py
│   │       ├── test/
│   │       │   └── test_serial_protocol.py
│   │       ├── package.xml
│   │       ├── setup.py
│   │       └── README.md
│   │
│   └── install/ (generated by colcon build)
│
├── hardware/
│   └── wiring_diagrams/
│
└── docs/
    ├── SETUP.md
    ├── FIRMWARE.md
    └── JOURNAL.md
```

## Summary

1. **Download hoverbot_driver package** (see next section)
2. **Extract to `ros2_ws/src/`**
3. **Build with colcon**
4. **Test hardware with test script**
5. **Launch driver**
6. **Test with teleop**
7. **Update bringup launch files** (optional)
8. **Commit and push to GitHub**

**The driver integrates seamlessly with your existing packages - no conflicts, no rewrites needed.**
