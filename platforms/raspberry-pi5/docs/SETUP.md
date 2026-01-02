# HoverBot Setup Guide

Complete step-by-step setup instructions from hardware to working robot.

---

## Prerequisites

### Required Hardware
- ✅ Hoverboard with single mainboard (STM32F103)
- ✅ Raspberry Pi 4 (4GB+ recommended)
- ✅ MicroSD card (32GB+ recommended)
- ✅ RPLidar A1
- ✅ 36V battery pack (10S Li-ion)
- ✅ ST-Link V2 programmer
- ✅ Dupont wires for UART connection

### Required Software (Development PC)
- Windows PC with STM32 ST-LINK Utility (for firmware flashing)
- Git
- VS Code with PlatformIO extension

---

## Step 1: Firmware Preparation

### 1.1 Clone Firmware Repository

```bash
# On Windows development PC
git clone https://github.com/EFeru/hoverboard-firmware-hack-FOC.git
cd hoverboard-firmware-hack-FOC
```

### 1.2 Configure Firmware

Edit `Inc/config.h`:

```c
/* ================= CRITICAL SETTINGS ================= */

// Power management (MUST BE 1 for proper power button)
#define BOARD_VARIANT 1

// Enable USART control
#define VARIANT_USART

// Serial configuration
#define CONTROL_SERIAL_USART3 0      // Right sideboard
#define FEEDBACK_SERIAL_USART3        // Enable telemetry
#define USART3_BAUD 115200
#define SERIAL_START_FRAME 0xABCD
#define SERIAL_TIMEOUT 160

// Drive mode
#define TANK_STEERING                 // Differential drive

// Input ranges
#define PRI_INPUT1  3, -1000, 0, 1000, 0
#define PRI_INPUT2  3, -1000, 0, 1000, 0
```

### 1.3 Build Firmware

**Option A: PlatformIO (recommended)**
```bash
# Install PlatformIO in VS Code
# Open project folder in VS Code
# Build: PlatformIO → Build (VARIANT_USART)
```

**Option B: Command line**
```bash
pio run -e VARIANT_USART
```

Output: `.pio/build/VARIANT_USART/firmware.bin`

---

## Step 2: Flash Firmware to Hoverboard

### 2.1 Hardware Setup

1. Connect ST-Link to hoverboard mainboard:
   - SWDIO → SWDIO
   - SWCLK → SWCLK  
   - GND → GND
   - 3.3V → 3.3V

2. Connect hoverboard battery

3. **Hold power button** (critical!)

### 2.2 Flash with STM32 ST-LINK Utility

1. Launch STM32 ST-LINK Utility
2. Target → Connect
3. Connection settings:
   - Mode: Normal
   - Speed: 100 kHz (more reliable)
   - Reset mode: Software
4. Target → Program & Verify
5. Browse to `.pio\build\VARIANT_USART\firmware.bin`
6. Start address: `0x08000000`
7. ☑ Verify after programming
8. **Keep holding power button**
9. Click "Start"
10. Wait for completion
11. Disconnect ST-Link
12. Release power button
13. Press power button normally

**Expected:** Single beep, board stays powered on

---

## Step 3: Raspberry Pi Setup

### 3.1 Install Ubuntu 24.04

1. Download [Ubuntu 24.04 LTS (64-bit) for Raspberry Pi](https://ubuntu.com/download/raspberry-pi)
2. Flash to microSD using Raspberry Pi Imager
3. Boot Pi and complete initial setup
4. Update system:
```bash
sudo apt update
sudo apt upgrade -y
```

### 3.2 Enable UART

Edit boot configuration:
```bash
sudo nano /boot/firmware/config.txt
```

Add these lines:
```
enable_uart=1
dtoverlay=disable-bt
```

Reboot:
```bash
sudo reboot
```

After reboot, configure serial:
```bash
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
sudo usermod -aG dialout $USER
```

Log out and back in for group changes to take effect.

### 3.3 Verify UART

```bash
# Check device exists
ls -l /dev/ttyAMA0
# Should show: crw-rw---- 1 root dialout

# Check baud rate
stty -F /dev/ttyAMA0
# Should include: speed 115200 baud
```

---

## Step 4: Wire Pi to Hoverboard

### 4.1 Right Sideboard Connection

**Hoverboard right sideboard connector pinout:**
```
Pin 1: VCC (not used)
Pin 2: GND
Pin 3: TX (from hoverboard)
Pin 4: RX (to hoverboard)
```

**Raspberry Pi GPIO (looking at board):**
```
Pi Pin 6  (GND)        → Sideboard Pin 2 (GND)
Pi Pin 8  (GPIO14 TX)  → Sideboard Pin 4 (RX)
Pi Pin 10 (GPIO15 RX)  → Sideboard Pin 3 (TX)
```

**Important:**
- Use right sideboard (USART3 is 5V tolerant)
- Double-check TX→RX and RX→TX crossover
- Ensure solid GND connection

### 4.2 Test Connection

Power on hoverboard (you should hear 3-beep pattern = waiting for serial commands).

---

## Step 5: Install ROS 2 Jazzy

### 5.1 Add ROS 2 Repository

```bash
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 5.2 Install ROS 2 Jazzy

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y
```

### 5.3 Install Development Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 5.4 Setup Environment

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify installation:
```bash
ros2 --version
# Should output: ros2 run X.Y.Z
```

---

## Step 6: Clone and Build HoverBot

### 6.1 Clone Repository

```bash
cd ~
git clone https://github.com/YOUR_USERNAME/hoverbot.git
cd hoverbot
```

### 6.2 Install Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 6.3 Install Python Dependencies

```bash
pip3 install pyserial
```

### 6.4 Build Workspace

```bash
colcon build --symlink-install
```

### 6.5 Source Workspace

```bash
source install/setup.bash
echo "source ~/hoverbot/install/setup.bash" >> ~/.bashrc
```

---

## Step 7: Test Serial Communication

### 7.1 Set Baud Rate

```bash
stty -F /dev/ttyAMA0 115200 raw -echo
```

### 7.2 Run Test Script

```bash
cd ~/hoverbot
python3 scripts/test_serial.py
```

**Expected output:**
```
Sending commands - beeping should STOP!

Sent: steer=   0 speed=   0 | CS=0xABCD
...
✓ Beeping stopped? (y/n): y
```

### 7.3 Motor Test

**⚠️ WHEELS OFF GROUND!**

```bash
python3 scripts/test_serial.py
```

Follow prompts to test forward/backward/turning.

---

## Step 8: Launch ROS 2 Visualization

### 8.1 Launch Robot State Publisher

Terminal 1:
```bash
ros2 launch hoverbot_bringup state_publisher.launch.py
```

### 8.2 Launch RViz

Terminal 2:
```bash
rviz2
```

In RViz:
- Add → RobotModel
- Fixed Frame: `base_link`
- You should see hoverbot model

---

## Troubleshooting

### Power button doesn't latch
- **Check:** `BOARD_VARIANT` in config.h
- **Should be:** `1` not `0`
- **Fix:** Change to 1, rebuild, reflash

### Continuous beeping
- **Cause:** Serial timeout (not receiving commands)
- **Check:** UART wiring (especially TX/RX crossover)
- **Check:** Baud rate set to 115200
- **Check:** Sending commands continuously (50Hz)

### Motors don't move
- **Check:** TANK_STEERING enabled in firmware
- **Check:** Wheels off ground
- **Check:** Battery voltage >36V
- **Try:** Lower speeds first (±100)

### UART permission denied
- **Fix:** `sudo usermod -aG dialout $USER`
- **Then:** Log out and back in

### ROS 2 build errors
- **Check:** All dependencies installed
- **Run:** `rosdep install --from-paths src --ignore-src -r -y`
- **Try:** `colcon build --symlink-install --cmake-clean-cache`

---

## Next Steps

Now that your robot is operational:

1. **Develop ROS 2 driver:** Create `hoverbot_driver` package
2. **Integrate RPLidar:** Add laser scanner to robot
3. **Test SLAM:** Build maps with slam_toolbox
4. **Enable Nav2:** Autonomous navigation

See main README.md for detailed roadmap.

---

## Safety Checklist

Before every operation:

- [ ] Wheels off ground for motor tests
- [ ] Battery voltage checked (>36V)
- [ ] Emergency stop ready (battery disconnect)
- [ ] Clear area around robot
- [ ] Serial communication working (no beeping)

---

## Reference

**Serial port:** `/dev/ttyAMA0`  
**Baud rate:** 115200  
**Command rate:** 50 Hz  
**Packet size:** 8 bytes  
**Checksum:** `start XOR steer XOR speed`

---

**Setup complete!** Your HoverBot should now be operational with working serial control.
