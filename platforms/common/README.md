# Common Platform Utilities

This directory contains shared utilities and templates that work across all supported platforms (Raspberry Pi 4, Raspberry Pi 5, Jetson Nano).

---

## Directory Structure

```
platforms/common/
├── scripts/
│   ├── test_serial.py          # Generic serial port tester
│   └── install_ros2.sh         # Common ROS 2 installation steps
├── config/
│   └── hoverbot_driver_template.yaml  # Template configuration
└── README.md                   # This file
```

---

## Supported Platforms

| Platform | Serial Device | ROS 2 Distribution | Ubuntu Version | Status |
|----------|---------------|-------------------|----------------|--------|
| **Raspberry Pi 4** | `/dev/ttyAMA0` | Humble | 22.04 LTS | ✅ Supported |
| **Raspberry Pi 5** | `/dev/ttyAMA0` | Jazzy | 24.04 LTS | ✅ Supported |
| **Jetson Nano** | `/dev/ttyTHS1` | Humble | 20.04 (JetPack) | ✅ Supported |

---

## Using Common Scripts

### Serial Port Testing

Test your serial connection on any platform:

```bash
# Generic test - auto-detect platform
cd ~/hoverbot/platforms/common/scripts
python3 test_serial.py --port /dev/ttyAMA0    # Pi4/Pi5
python3 test_serial.py --port /dev/ttyTHS1    # Jetson Nano
```

---

## Platform-Specific Directories

For platform-specific setup scripts and documentation, see:

- **Raspberry Pi 4**: `../raspberry-pi4/`
- **Raspberry Pi 5**: `../raspberry-pi5/`
- **Jetson Nano**: `../jetson-nano/`

---

## Configuration Templates

The `config/` directory contains template configuration files:

### `hoverbot_driver_template.yaml`

This is a template for the HoverBot driver configuration. Copy to your platform-specific directory and customize:

```bash
# For Pi4
cp platforms/common/config/hoverbot_driver_template.yaml \
   platforms/raspberry-pi4/config/hoverbot_driver.yaml

# Edit serial_port for your platform
nano platforms/raspberry-pi4/config/hoverbot_driver.yaml
```

---

## Key Differences Between Platforms

### Serial Port Device Paths

| Platform | Device | Why Different? |
|----------|--------|----------------|
| Raspberry Pi | `/dev/ttyAMA0` | PL011 UART on GPIO 14/15 |
| Jetson Nano | `/dev/ttyTHS1` | Tegra High-Speed UART 1 |

### ROS 2 Distribution Selection

| Platform | Ubuntu | ROS 2 | Reason |
|----------|--------|-------|--------|
| Pi 4 | 22.04 | Humble | LTS with best Pi4 support |
| Pi 5 | 24.04 | Jazzy | Native support for Pi5 hardware |
| Jetson | 20.04 | Humble | JetPack compatibility |

---

## Platform Abstraction in Code

All ROS 2 code in `ros2_ws/` is **platform-independent**. The serial port is parameterized:

```python
# In hoverbot_driver_node.py - works on ALL platforms
self.declare_parameter('serial_port', '/dev/ttyAMA0')  # Default
serial_port = self.get_parameter('serial_port').value
```

Override via launch file:

```bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py serial_port:=/dev/ttyTHS1
```

Or via config file:

```bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=/home/user/hoverbot/platforms/jetson-nano/config/hoverbot_driver.yaml
```

---

## Adding a New Platform

To add support for a new platform:

1. Create directory: `platforms/new-platform/`
2. Copy template config: `cp platforms/common/config/hoverbot_driver_template.yaml platforms/new-platform/config/`
3. Update serial port in config
4. Create platform-specific setup scripts (UART, dependencies)
5. Write `platforms/new-platform/docs/SETUP.md`
6. Update `docs/PLATFORM_SETUP_GUIDE.md`

---

## Common Installation Steps

All platforms share these steps:

1. **Install Ubuntu** (version varies by platform)
2. **Install ROS 2** (distribution varies by platform)
3. **Configure UART** (device path varies by platform)
4. **Clone repository**
5. **Build ROS 2 workspace** (same for all platforms)
6. **Flash hoverboard firmware** (same for all platforms)

Only steps 1-3 are platform-specific. Everything else is identical.

---

## Quick Platform Selection

Choose your platform:

- 👉 [**Raspberry Pi 4 Setup**](../raspberry-pi4/docs/SETUP.md)
- 👉 [**Raspberry Pi 5 Setup**](../raspberry-pi5/docs/SETUP.md)
- 👉 [**Jetson Nano Setup**](../jetson-nano/docs/SETUP.md)

---

**Need help?** See the main [Platform Setup Guide](../../docs/PLATFORM_SETUP_GUIDE.md)
