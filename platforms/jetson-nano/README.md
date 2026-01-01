# HoverBot on Jetson Nano

Complete setup guide for running HoverBot on NVIDIA Jetson Nano with JetPack and ROS 2 Humble.

---

## Platform Specifications

| Specification | Value |
|---------------|-------|
| **Platform** | NVIDIA Jetson Nano (4GB) |
| **Operating System** | JetPack 4.6.x or 5.x (Ubuntu 20.04) |
| **ROS 2 Distribution** | Humble Hawksbill |
| **Serial Port** | `/dev/ttyTHS1` (UART1 on J41 header) |
| **Baud Rate** | 115200 |
| **Status** | ✅ Supported (Not yet tested) |

---

## Quick Start

```bash
# 1. Flash JetPack to SD card
# 2. Boot Jetson and complete setup
# 3. Clone repository
git clone https://github.com/yourusername/hoverbot.git
cd hoverbot

# 4. Run UART setup
cd platforms/jetson-nano/scripts
chmod +x setup_uart.sh
./setup_uart.sh

# 5. Log out and back in (for group change)

# 6. Install ROS 2 Humble
# See docs for ROS 2 installation on Jetson

# 7. Build workspace
cd ~/hoverbot/ros2_ws
colcon build
source install/setup.bash

# 8. Launch with Jetson config
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=$HOME/hoverbot/platforms/jetson-nano/config/hoverbot_driver.yaml
```

---

## Directory Structure

```
platforms/jetson-nano/
├── config/
│   └── hoverbot_driver.yaml      # Jetson-specific configuration
├── scripts/
│   ├── setup_uart.sh             # UART setup script
│   └── install_dependencies.sh   # JetPack package installation (TODO)
├── docs/
│   └── SETUP.md                  # Comprehensive setup guide (TODO)
└── README.md                     # This file
```

---

## Configuration File

The Jetson configuration uses `/dev/ttyTHS1` for serial communication:

**File:** `platforms/jetson-nano/config/hoverbot_driver.yaml`

```yaml
hoverbot_driver:
  ros__parameters:
    serial_port: '/dev/ttyTHS1'  # Jetson Tegra High-Speed UART1
    baud_rate: 115200
    wheel_diameter: 0.165
    wheelbase: 0.40
    max_rpm: 300
```

---

## Hardware Connections

### UART Wiring (Jetson ↔ Hoverboard)

```
Jetson Nano (J41 Header)    Hoverboard Right Sideboard
------------------------    -------------------------
Pin  6 (GND)          →     Pin 2 (GND)
Pin  8 (UART1_TX)     →     Pin 4 (RX)
Pin 10 (UART1_RX)     →     Pin 3 (TX)
```

**J41 Header Pin Reference:**
- **Pin 8**: UART1_TX (GPIO14) → Hoverboard RX
- **Pin 10**: UART1_RX (GPIO15) → Hoverboard TX
- **Pin 6**: Ground

**⚠️ IMPORTANT:** Jetson Nano uses `/dev/ttyTHS1`, NOT `/dev/ttyAMA0`!

---

## Why Jetson Nano?

| Reason | Benefit |
|--------|---------|
| **GPU Acceleration** | NVIDIA 128-core Maxwell GPU |
| **AI Capabilities** | Run vision models, object detection |
| **Performance** | Quad-core ARM Cortex-A57 @ 1.43 GHz |
| **Power Efficiency** | 5-10W typical, excellent performance/watt |
| **Future Expansion** | Add cameras, computer vision, ML models |

**Use Case:** Choose Jetson if you plan to add:
- Object detection
- Lane following
- Person tracking
- Neural network-based navigation

---

## Performance Expectations

### Jetson Nano vs Raspberry Pi

| Metric | Jetson Nano | Pi 4 | Pi 5 |
|--------|------------|------|------|
| CPU | Cortex-A57 @ 1.43 GHz | Cortex-A72 @ 1.5 GHz | Cortex-A76 @ 2.4 GHz |
| GPU | 128-core Maxwell | None | VideoCore VII |
| RAM | 4GB LPDDR4 | 4GB LPDDR4 | 8GB LPDDR4X |
| AI Performance | Excellent (GPU) | Poor | Good |
| ROS 2 Performance | Good | Good | Excellent |
| Power | 5-10W | 5-7W | 8-10W |

**Bottom Line:** Jetson excels at vision/AI tasks. For basic navigation, Pi 5 is simpler.

---

## Troubleshooting

### Serial Port Not Found

```bash
# Check if ttyTHS1 exists
ls -l /dev/ttyTHS1

# If not found, check kernel logs
dmesg | grep ttyTHS

# Verify UART1 is enabled
cat /proc/device-tree/serial@70006040/status
# Should show: "okay"
```

### Permission Denied on Serial Port

```bash
# Check group membership
groups

# Should include 'dialout'
# If not, run setup script again:
cd ~/hoverbot/platforms/jetson-nano/scripts
./setup_uart.sh
```

### Serial Communication Test

```bash
# Configure baud rate
sudo stty -F /dev/ttyTHS1 115200 raw -echo

# Test with serial script
cd ~/hoverbot/platforms/common/scripts
python3 test_serial.py --port /dev/ttyTHS1
```

---

## Key Differences from Raspberry Pi

| Aspect | Raspberry Pi | Jetson Nano |
|--------|--------------|-------------|
| **Serial Device** | `/dev/ttyAMA0` | `/dev/ttyTHS1` |
| **UART Type** | PL011 | Tegra High-Speed |
| **Boot Config** | `/boot/firmware/config.txt` | Device Tree overlays |
| **Serial Service** | `serial-getty@ttyAMA0` | `nvgetty` |
| **OS** | Ubuntu 22.04/24.04 | JetPack (Ubuntu 20.04) |
| **Package Manager** | Standard apt | NVIDIA repositories |

---

## Installing ROS 2 on Jetson

ROS 2 Humble installation on Jetson requires source build:

```bash
# Install dependencies
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Create workspace and build ROS 2 from source
# See: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
```

**Alternative:** Use Docker container with ROS 2 Humble pre-installed.

---

## Future Enhancements

The Jetson platform enables:

1. **Computer Vision**
   - Object detection (YOLOv5/v8)
   - Lane detection
   - Obstacle recognition

2. **AI Navigation**
   - Neural network path planning
   - Reinforcement learning
   - End-to-end learning

3. **Advanced Perception**
   - SLAM with visual odometry
   - Semantic mapping
   - People detection and tracking

---

## Documentation

- **Comprehensive Setup:** [docs/SETUP.md](docs/SETUP.md) (TODO)
- **Hardware Wiring:** [../../hardware/README.md](../../hardware/README.md)
- **Firmware Guide:** [../../docs/FIRMWARE.md](../../docs/FIRMWARE.md)
- **Common Platform Guide:** [../common/README.md](../common/README.md)

---

## Support & Community

- **Issues:** https://github.com/yourusername/hoverbot/issues
- **Discussions:** https://github.com/yourusername/hoverbot/discussions
- **Platform Comparison:** [../../docs/PLATFORM_SETUP_GUIDE.md](../../docs/PLATFORM_SETUP_GUIDE.md)

---

**⚠️ Note:** Jetson Nano support is implemented but not yet tested on hardware. Contributions welcome!
