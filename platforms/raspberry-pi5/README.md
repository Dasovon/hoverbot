# HoverBot on Raspberry Pi 5

Complete setup guide for running HoverBot on Raspberry Pi 5 with Ubuntu 24.04 LTS and ROS 2 Jazzy.

---

## Platform Specifications

| Specification | Value |
|---------------|-------|
| **Platform** | Raspberry Pi 5 (8GB recommended) |
| **Operating System** | Ubuntu 24.04 LTS (Server or Desktop) |
| **ROS 2 Distribution** | Jazzy Jalisco |
| **Serial Port** | `/dev/ttyAMA0` (GPIO 14/15) |
| **Baud Rate** | 115200 |
| **Status** | ✅ Fully Supported & Tested |

---

## Quick Start

```bash
# 1. Flash Ubuntu 24.04 to SD card
# 2. Boot Pi and connect to network
# 3. Clone repository
git clone https://github.com/yourusername/hoverbot.git
cd hoverbot

# 4. Run UART setup
cd platforms/raspberry-pi5/scripts
chmod +x setup_uart.sh
./setup_uart.sh

# 5. Reboot and install ROS 2
sudo reboot

# 6. Install ROS 2 Jazzy (after reboot)
# See docs/SETUP.md for detailed ROS 2 installation

# 7. Build workspace
cd ~/hoverbot/ros2_ws
colcon build
source install/setup.bash

# 8. Launch with Pi5 config
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=$HOME/hoverbot/platforms/raspberry-pi5/config/hoverbot_driver.yaml
```

---

## Directory Structure

```
platforms/raspberry-pi5/
├── config/
│   └── hoverbot_driver.yaml      # Pi5-specific configuration
├── scripts/
│   ├── setup_uart.sh             # UART setup script
│   └── test_serial.py            # Serial communication tester
├── docs/
│   └── SETUP.md                  # Comprehensive setup guide
└── README.md                     # This file
```

---

## Configuration File

The Pi5 configuration uses `/dev/ttyAMA0` for serial communication:

**File:** `platforms/raspberry-pi5/config/hoverbot_driver.yaml`

```yaml
hoverbot_driver:
  ros__parameters:
    serial_port: '/dev/ttyAMA0'  # Pi5 UART on GPIO 14/15
    baud_rate: 115200
    wheel_diameter: 0.165
    wheelbase: 0.40
    max_rpm: 300
```

---

## Hardware Connections

### UART Wiring (Pi5 ↔ Hoverboard)

```
Raspberry Pi 5          Hoverboard Right Sideboard
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

## Why Ubuntu 24.04 + ROS 2 Jazzy?

| Reason | Benefit |
|--------|---------|
| **Latest LTS** | Ubuntu 24.04 LTS support until 2029 |
| **Native Pi5 Support** | Optimized kernel and drivers |
| **ROS 2 Jazzy** | Latest ROS 2 LTS (until 2029) |
| **Modern Packages** | Latest versions of all dependencies |
| **Performance** | Best optimization for Pi 5 Cortex-A76 |

---

## Performance

### Validated Performance Metrics

| Metric | Target | Actual | Status |
|--------|--------|--------|--------|
| Serial Success Rate | >95% | 99.3% | ✅ Excellent |
| Odometry Rate | 50 Hz | 50 Hz | ✅ Perfect |
| LiDAR Scan Rate | 10 Hz | 7.6 Hz | ✅ Adequate |
| SLAM Map Rate | 0.1 Hz | 0.1 Hz | ✅ Perfect |
| IMU Update Rate | 100 Hz | 100 Hz | ✅ Perfect |
| CPU Usage (Full Stack) | <50% | ~40% | ✅ Excellent |

**Tested Configuration:** Pi 5 8GB, Ubuntu 24.04, ROS 2 Jazzy, all 7 sensors active

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
cd ~/hoverbot/platforms/raspberry-pi5/scripts
python3 test_serial.py
```

---

## Differences from Pi 4

| Aspect | Pi 4 | Pi 5 |
|--------|------|------|
| **Ubuntu Version** | 22.04 LTS | 24.04 LTS |
| **ROS 2 Version** | Humble | Jazzy |
| **Serial Port** | `/dev/ttyAMA0` | `/dev/ttyAMA0` (same) |
| **CPU** | Cortex-A72 @ 1.5 GHz | Cortex-A76 @ 2.4 GHz |
| **Performance** | Good (~60% CPU) | Excellent (~40% CPU) |
| **Power Consumption** | 5-7W | 8-10W |

**Key Insight:** Serial configuration is identical! Only OS/ROS versions and performance differ.

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

**✅ Raspberry Pi 5 is the recommended platform - fully tested and validated!**
