# HoverBot on Raspberry Pi 4

Complete setup guide for running HoverBot on Raspberry Pi 4 with Ubuntu 22.04 LTS and ROS 2 Humble.

---

## Platform Specifications

| Specification | Value |
|---------------|-------|
| **Platform** | Raspberry Pi 4 (4GB/8GB recommended) |
| **Operating System** | Ubuntu 22.04 LTS (Server or Desktop) |
| **ROS 2 Distribution** | Humble Hawksbill |
| **Serial Port** | `/dev/ttyAMA0` (GPIO 14/15) |
| **Baud Rate** | 115200 |
| **Status** | ✅ Fully Supported |

---

## Quick Start

```bash
# 1. Flash Ubuntu 22.04 to SD card
# 2. Boot Pi and connect to network
# 3. Clone repository
git clone https://github.com/yourusername/hoverbot.git
cd hoverbot

# 4. Run UART setup
cd platforms/raspberry-pi4/scripts
chmod +x setup_uart.sh
./setup_uart.sh

# 5. Reboot and install ROS 2
sudo reboot

# 6. Install ROS 2 Humble (after reboot)
# See docs/SETUP.md for detailed ROS 2 installation

# 7. Build workspace
cd ~/hoverbot/ros2_ws
colcon build
source install/setup.bash

# 8. Launch with Pi4 config
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=$HOME/hoverbot/platforms/raspberry-pi4/config/hoverbot_driver.yaml
```

---

## Directory Structure

```
platforms/raspberry-pi4/
├── config/
│   └── hoverbot_driver.yaml      # Pi4-specific configuration
├── scripts/
│   ├── setup_uart.sh             # UART setup script
│   └── install_dependencies.sh   # Package installation (TODO)
├── docs/
│   └── SETUP.md                  # Comprehensive setup guide
└── README.md                     # This file
```

---

## Configuration File

The Pi4 configuration uses `/dev/ttyAMA0` for serial communication:

**File:** `platforms/raspberry-pi4/config/hoverbot_driver.yaml`

```yaml
hoverbot_driver:
  ros__parameters:
    serial_port: '/dev/ttyAMA0'  # Pi4 UART on GPIO 14/15
    baud_rate: 115200
    wheel_diameter: 0.165
    wheelbase: 0.40
    max_rpm: 300
```

---

## Hardware Connections

### UART Wiring (Pi4 ↔ Hoverboard)

```
Raspberry Pi 4          Hoverboard Right Sideboard
--------------          -------------------------
Pin  6 (GND)      →     Pin 2 (GND)
Pin  8 (GPIO14)   →     Pin 4 (RX)
Pin 10 (GPIO15)   →     Pin 3 (TX)
```

**GPIO Pin Reference:**
- **Pin 8 (GPIO14)**: UART TX → Hoverboard RX
- **Pin 10 (GPIO15)**: UART RX → Hoverboard TX
- **Pin 6**: Ground

---

## Why Ubuntu 22.04 + ROS 2 Humble?

| Reason | Benefit |
|--------|---------|
| **LTS Support** | Long-term support until 2027 |
| **ROS 2 Humble** | LTS ROS distribution (until 2027) |
| **Stability** | Well-tested on Pi 4 hardware |
| **Package Availability** | Excellent arm64 package support |
| **Performance** | Optimized for Pi 4 quad-core Cortex-A72 |

---

## Performance Expectations

### Pi 4 vs Pi 5 Performance

| Metric | Pi 4 (4GB) | Pi 5 (8GB) |
|--------|-----------|-----------|
| CPU | Cortex-A72 @ 1.5 GHz | Cortex-A76 @ 2.4 GHz |
| RAM | 4GB LPDDR4 | 8GB LPDDR4X |
| SLAM Update Rate | 0.1 Hz | 0.1 Hz |
| Odometry Rate | 50 Hz | 50 Hz |
| Expected Load | ~60% CPU | ~40% CPU |

**Bottom Line:** Pi 4 has sufficient performance for HoverBot navigation and SLAM.

---

## Troubleshooting

### Serial Port Not Found

```bash
# Check if ttyAMA0 exists
ls -l /dev/ttyAMA0

# Check UART configuration
sudo cat /boot/firmware/config.txt | grep uart

# Should see:
#   enable_uart=1
#   dtoverlay=disable-bt
```

### Permission Denied on Serial Port

```bash
# Check group membership
groups

# Should include 'dialout'
# If not, add yourself and reboot:
sudo usermod -aG dialout $USER
sudo reboot
```

### Serial Communication Test

```bash
# Configure baud rate
stty -F /dev/ttyAMA0 115200 raw -echo

# Test with serial script
cd ~/hoverbot/platforms/common/scripts
python3 test_serial.py --port /dev/ttyAMA0
```

---

## Differences from Pi 5

| Aspect | Pi 4 | Pi 5 |
|--------|------|------|
| **Ubuntu Version** | 22.04 LTS | 24.04 LTS |
| **ROS 2 Version** | Humble | Jazzy |
| **Serial Port** | `/dev/ttyAMA0` | `/dev/ttyAMA0` (same) |
| **Boot Config Path** | `/boot/firmware/config.txt` | `/boot/firmware/config.txt` (same) |
| **Performance** | Good | Excellent |
| **Power Consumption** | 5-7W | 8-10W |

**Key Insight:** Serial configuration is identical! Only OS and ROS versions differ.

---

## Upgrading to Pi 5

If you want to upgrade to Pi 5 later:

1. Flash Ubuntu 24.04 to new SD card
2. Install ROS 2 Jazzy instead of Humble
3. Use Pi5 configuration: `platforms/raspberry-pi5/config/hoverbot_driver.yaml`
4. Everything else stays the same!

---

## Documentation

- **Comprehensive Setup:** [docs/SETUP.md](docs/SETUP.md)
- **Hardware Wiring:** [../../hardware/README.md](../../hardware/README.md)
- **Firmware Guide:** [../../docs/FIRMWARE.md](../../docs/FIRMWARE.md)
- **Common Platform Guide:** [../common/README.md](../common/README.md)

---

## Support & Community

- **Issues:** https://github.com/yourusername/hoverbot/issues
- **Discussions:** https://github.com/yourusername/hoverbot/discussions
- **Platform Comparison:** [../../docs/PLATFORM_SETUP_GUIDE.md](../../docs/PLATFORM_SETUP_GUIDE.md)

---

**✅ Raspberry Pi 4 is fully supported and tested!**
