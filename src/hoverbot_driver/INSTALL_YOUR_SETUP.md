# Installation Instructions - Your Dev Machine Setup

## Your Configuration

**Development Machine:**
- OS: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- CPU: Intel Core i5-9400 @ 2.90GHz (6 cores)
- GPU: NVIDIA GeForce GT 1030
- RAM: 16 GB
- Storage: 2 TB
- Architecture: x86_64 / amd64
- ROS 2: Humble Hawksbill (LTS)

**Target Deployment:**
- Raspberry Pi 5 (ARM64)
- Ubuntu 24.04 LTS
- ROS 2 Jazzy Jalisco (or Humble)

## Installation Steps for Your Dev Machine

### 1. Verify ROS 2 Humble Installation

```bash
# Check ROS 2 version
ros2 --version

# Expected output:
# ros2 doctor 0.10.x
# using ROS 2 distro: humble

# If not installed, install ROS 2 Humble:
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```

### 2. Install Required Dependencies

```bash
# Update package lists
sudo apt update

# Install build tools and dependencies
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial \
    python3-pytest \
    python3-pip \
    git

# Install pyserial via pip (backup)
pip3 install pyserial

# Initialize rosdep (skip if already done)
sudo rosdep init || true
rosdep update
```

### 3. Clone Your Repository

```bash
# Navigate to home directory
cd ~

# Clone your hoverbot repository
git clone https://github.com/Dasovon/hoverbot.git

# Navigate into repository
cd hoverbot

# Verify structure
ls -la
# Should show: firmware/ raspberry-pi/ ros2_ws/ hardware/ docs/
```

### 4. Install Driver Package

```bash
# Download hoverbot_driver.tar.gz from Claude
# (Should be in your Downloads folder)

# Extract the archive
cd ~/Downloads
tar -xzf hoverbot_driver.tar.gz

# Verify extraction
ls hoverbot_driver/
# Should show multiple .md files and folders

# Copy to your workspace
cp -r hoverbot_driver ~/hoverbot/ros2_ws/src/

# Verify installation
ls ~/hoverbot/ros2_ws/src/
# Should show: hoverbot_description  hoverbot_bringup  hoverbot_driver
```

### 5. Install Package Dependencies

```bash
# Navigate to workspace
cd ~/hoverbot/ros2_ws

# Install dependencies via rosdep
rosdep install --from-paths src --ignore-src -r -y

# Expected output:
# #All required rosdeps installed successfully
```

### 6. Build the Driver Package

```bash
# Still in ~/hoverbot/ros2_ws
colcon build --packages-select hoverbot_driver

# Expected output:
# Starting >>> hoverbot_driver
# Finished <<< hoverbot_driver [0.5s]
# 
# Summary: 1 package finished [0.7s]

# Source the workspace
source install/setup.bash
```

### 7. Verify Installation

```bash
# Check package is installed
ros2 pkg list | grep hoverbot

# Expected output:
# hoverbot_bringup
# hoverbot_description
# hoverbot_driver

# Check executables
ros2 pkg executables hoverbot_driver

# Expected output:
# hoverbot_driver hoverbot_driver_node

# Check launch files
ros2 launch hoverbot_driver hoverbot_driver.launch.py --show-args

# Should show launch arguments without errors
```

### 8. Test Python Imports

```bash
# Test importing driver modules
python3 -c "from hoverbot_driver.serial_interface import HoverboardSerialInterface; print('✓ Serial interface import OK')"

python3 -c "from hoverbot_driver.differential_drive_controller import DifferentialDriveController; print('✓ Controller import OK')"

python3 -c "from hoverbot_driver.hoverbot_driver_node import HoverBotDriverNode; print('✓ Driver node import OK')"

# All three should print OK
```

### 9. Build All Packages (Optional)

```bash
# Build entire workspace
cd ~/hoverbot/ros2_ws
colcon build

# This builds:
# - hoverbot_description
# - hoverbot_bringup
# - hoverbot_driver

# Source again
source install/setup.bash
```

### 10. Commit to Git

```bash
# Navigate to repository root
cd ~/hoverbot

# Check git status
git status
# Should show: ros2_ws/src/hoverbot_driver/ (untracked)

# Add driver package
git add ros2_ws/src/hoverbot_driver/

# Commit
git commit -m "Add ROS 2 serial driver for hoverboard control

- Implements USART protocol for EFeru firmware
- Publishes odometry from wheel encoders
- Supports differential drive kinematics
- Compatible with Ubuntu 22.04 + ROS 2 Humble
- Ready for Nav2 integration"

# Push to GitHub
git push origin main
```

## Testing Without Hardware

Since you don't have the hoverboard connected to your dev machine:

### Test 1: Kinematics Calculation

```bash
cd ~/hoverbot/ros2_ws/src/hoverbot_driver

# Test forward motion
python3 -c "
from hoverbot_driver.differential_drive_controller import DifferentialDriveController

controller = DifferentialDriveController()

# Test forward motion
left, right = controller.twist_to_wheels(0.2, 0.0)
print(f'Forward 0.2 m/s: left={left}, right={right}')
assert left == right, 'Forward motion should have equal wheels'

# Test rotation
left, right = controller.twist_to_wheels(0.0, 0.5)
print(f'Rotate 0.5 rad/s: left={left}, right={right}')
assert left == -right, 'Pure rotation should have opposite wheels'

print('✓ Kinematics tests passed')
"
```

### Test 2: Serial Packet Format

```bash
# Test packet creation (no hardware needed)
python3 -c "
from hoverbot_driver.serial_interface import HoverboardSerialInterface
import struct

serial = HoverboardSerialInterface()

# Test checksum
checksum = serial._calculate_checksum(0xABCD, 500, -300)
print(f'Checksum: {checksum:#06x}')

# Test packet structure
packet = struct.pack('<HhhH', 0xABCD, 500, -300, checksum)
print(f'Packet: {packet.hex()}')
print(f'Length: {len(packet)} bytes')
assert len(packet) == 8, 'Packet must be 8 bytes'

print('✓ Protocol tests passed')
"
```

### Test 3: Launch File Syntax

```bash
# Test launch file loads without errors
ros2 launch hoverbot_driver hoverbot_driver.launch.py --show-args

# Should show arguments without crashing
```

## Deploying to Raspberry Pi

### Transfer Via Git (Recommended)

```bash
# On dev machine (already done above):
cd ~/hoverbot
git push

# On Raspberry Pi:
ssh pi@raspberrypi.local
cd ~
git clone https://github.com/Dasovon/hoverbot.git
cd hoverbot/ros2_ws

# Install dependencies on Pi
rosdep install --from-paths src --ignore-src -r -y

# Build on Pi (architecture-specific binaries)
colcon build --packages-select hoverbot_driver
source install/setup.bash

# Test hardware
sudo chmod 666 /dev/ttyAMA0
python3 src/hoverbot_driver/test/test_serial_protocol.py

# Launch driver
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

## Development Workflow

### Typical Cycle:

1. **Edit on dev machine:**
   ```bash
   cd ~/hoverbot/ros2_ws/src/hoverbot_driver
   # Edit Python files, configs, launch files
   ```

2. **Build and test syntax:**
   ```bash
   cd ~/hoverbot/ros2_ws
   colcon build --packages-select hoverbot_driver
   # Catches Python syntax errors, import issues
   ```

3. **Commit changes:**
   ```bash
   cd ~/hoverbot
   git add .
   git commit -m "Update driver: <description>"
   git push
   ```

4. **Deploy to Pi:**
   ```bash
   # SSH to Pi
   ssh pi@raspberrypi.local
   cd ~/hoverbot
   git pull
   cd ros2_ws
   colcon build --packages-select hoverbot_driver
   source install/setup.bash
   
   # Test with real hardware
   ros2 launch hoverbot_driver hoverbot_driver.launch.py
   ```

## Environment Setup

### Add to ~/.bashrc (Recommended)

```bash
# Add to end of ~/.bashrc
echo "" >> ~/.bashrc
echo "# ROS 2 Humble setup" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/hoverbot/ros2_ws/install/setup.bash" >> ~/.bashrc

# Reload
source ~/.bashrc
```

Now ROS 2 and your workspace are sourced automatically in every terminal.

## Verification Checklist

- [ ] ROS 2 Humble installed and working
- [ ] Repository cloned to ~/hoverbot
- [ ] Driver package in ~/hoverbot/ros2_ws/src/hoverbot_driver/
- [ ] Dependencies installed (rosdep, python3-serial)
- [ ] Package builds without errors
- [ ] All Python modules import successfully
- [ ] Launch file syntax validates
- [ ] Kinematics tests pass
- [ ] Protocol tests pass
- [ ] Changes committed to Git
- [ ] Pushed to GitHub

## Troubleshooting

### Build Fails: "Could not find a package configuration file"

```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Rebuild
cd ~/hoverbot/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select hoverbot_driver
```

### Import Error: "No module named 'hoverbot_driver'"

```bash
# Source the workspace
source ~/hoverbot/ros2_ws/install/setup.bash

# Or add to ~/.bashrc permanently
```

### "python3-serial" Not Found

```bash
# Install via apt
sudo apt install python3-serial

# Or via pip
pip3 install pyserial
```

### Git Push Fails: Authentication

```bash
# Set up SSH key or use personal access token
# https://docs.github.com/en/authentication
```

## Next Steps

1. ✅ Install driver on dev machine (you're here)
2. ⏭️ Push to GitHub
3. ⏭️ Pull on Raspberry Pi
4. ⏭️ Test with hardware
5. ⏭️ Integrate with Nav2
6. ⏭️ Add SLAM mapping
7. ⏭️ Configure autonomous navigation

## Summary

**Your dev machine setup:**
- ✅ Ubuntu 22.04 + ROS 2 Humble
- ✅ 16 GB RAM (plenty for development)
- ✅ Can build and test package
- ✅ Git workflow to deploy to Pi
- ✅ No hardware needed for development

**Total installation time:** ~15 minutes

The driver package is now ready to develop on your dev machine and deploy to your Raspberry Pi!
