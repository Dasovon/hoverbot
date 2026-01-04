# Installation for Ubuntu 22.04 + ROS 2 Humble

## Your Development Setup

- **OS:** Ubuntu 22.04.5 LTS (Jammy)
- **ROS:** ROS 2 Humble Hawksbill (LTS)
- **Machine:** x86_64 dev machine (Intel i5-9400)
- **Memory:** 16 GB
- **Target:** Deploy to Raspberry Pi 5

## Important Notes

### Development Workflow
You're developing on **x86_64 Ubuntu 22.04**, but deploying to **ARM64 Raspberry Pi**.

**Recommended workflow:**
1. **Develop on your dev machine** (this guide)
2. **Test with simulation** (no serial port needed)
3. **Transfer to Pi** (clone repo, rebuild)
4. **Test with hardware** (actual hoverboard)

### ROS 2 Humble vs Jazzy

The driver package works with both, but:
- **Humble:** Ubuntu 22.04 LTS (your setup)
- **Jazzy:** Ubuntu 24.04 (Pi 5 requirement)

**No code changes needed** - package supports both!

## Installation Steps

### 1. Install ROS 2 Humble (If Not Already Installed)

```bash
# Check if ROS 2 Humble is installed
ros2 --version
# Should show: ros2 humble <version>

# If not installed, follow:
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```

### 2. Install Dependencies

```bash
# System dependencies
sudo apt update
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial \
    python3-pytest

# Initialize rosdep (if first time)
sudo rosdep init  # Skip if already done
rosdep update
```

### 3. Clone Your Repository

```bash
# Clone your hoverbot repo
cd ~
git clone https://github.com/Dasovon/hoverbot.git
cd hoverbot
```

### 4. Extract and Install Driver Package

```bash
# Extract the driver package (downloaded from Claude)
cd ~/Downloads
tar -xzf hoverbot_driver.tar.gz

# Copy to your workspace
cp -r hoverbot_driver ~/hoverbot/ros2_ws/src/

# Verify
ls ~/hoverbot/ros2_ws/src/
# Should show: hoverbot_description  hoverbot_bringup  hoverbot_driver
```

### 5. Install Package Dependencies

```bash
cd ~/hoverbot/ros2_ws

# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y

# Manually install pyserial (just in case)
pip3 install pyserial
```

### 6. Build the Package

```bash
cd ~/hoverbot/ros2_ws

# Build only the driver package first
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

### 7. Verify Installation

```bash
# Check package is available
ros2 pkg list | grep hoverbot

# Should show:
#   hoverbot_bringup
#   hoverbot_description
#   hoverbot_driver

# Check executables
ros2 pkg executables hoverbot_driver
# Should show: hoverbot_driver hoverbot_driver_node
```

## Development Machine Testing (No Hardware)

Since you don't have the hoverboard connected to your dev machine, you can:

### Option 1: Test with Mock Serial Port

Create a virtual serial port pair for testing:

```bash
# Install socat for virtual serial ports
sudo apt install socat

# Create virtual serial port pair
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# Output will show: N PTY is /dev/pts/X and /dev/pts/Y

# In another terminal, launch driver with virtual port
ros2 launch hoverbot_driver hoverbot_driver.launch.py serial_port:=/dev/pts/X
```

### Option 2: Build Everything and Transfer to Pi

```bash
cd ~/hoverbot/ros2_ws

# Build all packages
colcon build

# Package workspace for transfer to Pi
tar -czf ros2_ws.tar.gz .

# Transfer to Pi (replace with your Pi's IP)
scp ros2_ws.tar.gz pi@raspberrypi.local:~/
```

### Option 3: Simulation Mode (Future Enhancement)

For now, the driver requires serial connection. Future versions could add simulation mode.

## Deploying to Raspberry Pi

### On Your Dev Machine:

```bash
# Commit driver to your repo
cd ~/hoverbot
git add ros2_ws/src/hoverbot_driver/
git commit -m "Add ROS 2 driver for hoverboard control"
git push
```

### On Your Raspberry Pi:

```bash
# Pull latest code
cd ~/hoverbot
git pull

# Rebuild on Pi (architecture-specific)
cd ros2_ws
colcon build --packages-select hoverbot_driver
source install/setup.bash

# Test hardware
python3 src/hoverbot_driver/test/test_serial_protocol.py

# Launch driver
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

## Development Workflow

### Typical Development Cycle:

1. **Edit code on dev machine** (your x86_64 Ubuntu 22.04)
   ```bash
   cd ~/hoverbot/ros2_ws/src/hoverbot_driver
   # Edit files
   ```

2. **Build and test syntax**
   ```bash
   cd ~/hoverbot/ros2_ws
   colcon build --packages-select hoverbot_driver
   ```

3. **Commit and push**
   ```bash
   cd ~/hoverbot
   git add .
   git commit -m "Update driver"
   git push
   ```

4. **Pull and test on Pi**
   ```bash
   # SSH to Pi
   ssh pi@raspberrypi.local
   cd ~/hoverbot
   git pull
   cd ros2_ws
   colcon build --packages-select hoverbot_driver
   source install/setup.bash
   ros2 launch hoverbot_driver hoverbot_driver.launch.py
   ```

## Cross-Compilation (Advanced)

If you want to build ARM binaries on your x86_64 machine:

```bash
# Install cross-compilation tools
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# This is complex for ROS 2 and usually not worth it
# Easier to just build on the Pi
```

## ROS 2 Humble Specifics

### Package Compatibility

All ROS 2 packages are compatible with Humble:
- ✅ `geometry_msgs` - Same in Humble and Jazzy
- ✅ `nav_msgs` - Same in Humble and Jazzy
- ✅ `tf2_ros` - Same in Humble and Jazzy
- ✅ Python API - Compatible across versions

### Known Issues with Humble

**None!** The driver uses only stable APIs that work in:
- ROS 2 Humble (Ubuntu 22.04)
- ROS 2 Jazzy (Ubuntu 24.04)
- Future ROS 2 versions

## Testing Without Hardware

### Unit Tests (Python)

```bash
cd ~/hoverbot/ros2_ws/src/hoverbot_driver

# Test kinematics
python3 -c "
from hoverbot_driver.differential_drive_controller import DifferentialDriveController
controller = DifferentialDriveController()
left, right = controller.twist_to_wheels(0.2, 0.0)
print(f'Forward 0.2 m/s: left={left}, right={right}')
"
```

### Mock Serial Interface

```bash
cd ~/hoverbot/ros2_ws/src/hoverbot_driver

# Test serial packet creation (no hardware needed)
python3 -c "
from hoverbot_driver.serial_interface import HoverboardSerialInterface
import struct

serial = HoverboardSerialInterface()
# Test checksum calculation
checksum = serial._calculate_checksum(0xABCD, 100, 200)
print(f'Checksum for (100, 200): {checksum:#06x}')

# Test packet creation
packet = struct.pack('<HhhH', 0xABCD, 100, 200, checksum)
print(f'Packet length: {len(packet)} bytes')
assert len(packet) == 8
print('✓ Packet format valid')
"
```

## Troubleshooting on Dev Machine

### Build Errors

```bash
# Clean rebuild
cd ~/hoverbot/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select hoverbot_driver
```

### Import Errors

```bash
# Make sure workspace is sourced
source ~/hoverbot/ros2_ws/install/setup.bash

# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))"
# Should include: .../ros2_ws/install/hoverbot_driver/...
```

### ROS 2 Not Found

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc for permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/hoverbot/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Architecture Differences Summary

| Aspect | Dev Machine (x86_64) | Raspberry Pi (ARM64) |
|--------|---------------------|---------------------|
| OS | Ubuntu 22.04 | Ubuntu 24.04 |
| ROS 2 | Humble | Jazzy (or Humble) |
| Build | Native x86_64 | Native ARM64 |
| Testing | Simulation/Mock | Real Hardware |
| Serial Port | Virtual/None | /dev/ttyAMA0 |

**Key Point:** Build separately on each architecture. Don't copy binaries between machines.

## Verification Checklist

On your dev machine:
- [ ] ROS 2 Humble installed
- [ ] Repository cloned
- [ ] Driver package extracted to src/
- [ ] Dependencies installed (rosdep, pyserial)
- [ ] Package builds without errors
- [ ] Can import Python modules
- [ ] Ready to commit to Git

On Raspberry Pi (later):
- [ ] Pull latest from Git
- [ ] Rebuild on Pi
- [ ] Test hardware with test script
- [ ] Launch driver successfully
- [ ] Wheels respond to commands

## Next Steps

1. **On dev machine:**
   - Install and build driver
   - Commit to Git
   - Develop launch files and configs

2. **On Raspberry Pi:**
   - Pull from Git
   - Rebuild on Pi
   - Test with hardware
   - Deploy and run

## Summary

**Your Development Environment:**
- ✅ Ubuntu 22.04 + ROS 2 Humble
- ✅ x86_64 architecture
- ✅ 16 GB RAM (plenty for development)
- ✅ No hardware needed for development

**Deployment Environment:**
- Raspberry Pi 5 (ARM64)
- Ubuntu 24.04 + ROS 2 Jazzy (or Humble)
- Actual hoverboard hardware
- Real serial communication

**The driver package works on both!** Just build on each platform separately.
