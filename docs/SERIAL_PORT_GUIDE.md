# Serial Port Configuration Guide

**Cross-platform serial communication setup for HoverBot**

This guide explains how serial port configuration works across different platforms and how to configure HoverBot for your specific hardware.

---

## Platform Serial Port Summary

| Platform | Serial Device | UART Type | GPIO Pins | Setup Difficulty |
|----------|---------------|-----------|-----------|------------------|
| **Raspberry Pi 4** | `/dev/ttyAMA0` | PL011 | GPIO 14/15 | ⭐ Easy |
| **Raspberry Pi 5** | `/dev/ttyAMA0` | PL011 | GPIO 14/15 | ⭐ Easy |
| **Jetson Nano** | `/dev/ttyTHS1` | Tegra HS UART | J41 pins 8/10 | ⭐⭐ Moderate |

---

## Understanding Serial Devices

### Raspberry Pi: `/dev/ttyAMA0`

**What is it?**
- PL011 UART (ARM's standard UART peripheral)
- Hardware UART with full feature support
- Located on GPIO pins 14 (TX) and 15 (RX)

**Why ttyAMA0?**
- `tty` = Terminal device
- `AMA` = ARM Microcontroller Architecture (PL011 UART)
- `0` = First UART device

**Key Features:**
- ✅ Hardware UART (reliable, low CPU usage)
- ✅ Same on both Pi 4 and Pi 5
- ✅ Full RS-232 compatible
- ✅ 3.3V logic level

### Jetson Nano: `/dev/ttyTHS1`

**What is it?**
- Tegra High-Speed UART 1
- Built into Tegra SoC
- Located on J41 header pins 8 (TX) and 10 (RX)

**Why ttyTHS1?**
- `tty` = Terminal device
- `THS` = Tegra High-Speed UART
- `1` = UART1 (there are multiple UARTs)

**Key Features:**
- ✅ High-speed hardware UART
- ✅ Optimized for Tegra SoC
- ⚠️ Different from Raspberry Pi
- ✅ 3.3V logic level

---

## Physical Pin Mapping

### Raspberry Pi 4/5 (40-pin GPIO Header)

```
         3.3V  [ 1] [ 2]  5V
        GPIO2  [ 3] [ 4]  5V
        GPIO3  [ 5] [ 6]  GND  ← Connect to Hoverboard GND
        GPIO4  [ 7] [ 8]  GPIO14 (TXD) ← Connect to Hoverboard RX
          GND  [ 9] [10]  GPIO15 (RXD) ← Connect to Hoverboard TX
       GPIO17  [11] [12]  GPIO18
       ...
```

**HoverBot Connections:**
- **Pin 6 (GND)** → Hoverboard GND
- **Pin 8 (GPIO14/TXD)** → Hoverboard RX (crossover!)
- **Pin 10 (GPIO15/RXD)** → Hoverboard TX (crossover!)

### Jetson Nano (J41 40-pin Header)

```
         3.3V  [ 1] [ 2]  5V
   I2C1_SDA  [ 3] [ 4]  5V
   I2C1_SCL  [ 5] [ 6]  GND  ← Connect to Hoverboard GND
     GPIO216  [ 7] [ 8]  UART1_TX ← Connect to Hoverboard RX
          GND  [ 9] [10]  UART1_RX ← Connect to Hoverboard TX
     GPIO50  [11] [12]  I2S0_SCLK
       ...
```

**HoverBot Connections:**
- **Pin 6 (GND)** → Hoverboard GND
- **Pin 8 (UART1_TX)** → Hoverboard RX (crossover!)
- **Pin 10 (UART1_RX)** → Hoverboard TX (crossover!)

**Note:** Same physical pins (6, 8, 10) on both platforms!

---

## Configuration Files

### Raspberry Pi 4

**File:** `platforms/raspberry-pi4/config/hoverbot_driver.yaml`

```yaml
hoverbot_driver:
  ros__parameters:
    serial_port: '/dev/ttyAMA0'  # Pi4 UART
    baud_rate: 115200
```

### Raspberry Pi 5

**File:** `platforms/raspberry-pi5/config/hoverbot_driver.yaml`

```yaml
hoverbot_driver:
  ros__parameters:
    serial_port: '/dev/ttyAMA0'  # Pi5 UART (same as Pi4!)
    baud_rate: 115200
```

### Jetson Nano

**File:** `platforms/jetson-nano/config/hoverbot_driver.yaml`

```yaml
hoverbot_driver:
  ros__parameters:
    serial_port: '/dev/ttyTHS1'  # Jetson Tegra UART
    baud_rate: 115200
```

---

## Enabling UART

### Raspberry Pi 4/5

1. **Edit boot configuration:**
   ```bash
   sudo nano /boot/firmware/config.txt
   ```

2. **Add these lines:**
   ```
   enable_uart=1
   dtoverlay=disable-bt
   ```

3. **Disable serial console:**
   ```bash
   sudo systemctl disable serial-getty@ttyAMA0.service
   ```

4. **Reboot:**
   ```bash
   sudo reboot
   ```

### Jetson Nano

1. **Disable nvgetty service:**
   ```bash
   sudo systemctl disable nvgetty
   ```

2. **Add udev rule:**
   ```bash
   echo 'KERNEL=="ttyTHS1", MODE="0666"' | sudo tee /etc/udev/rules.d/99-hoverbot-uart.rules
   sudo udevadm control --reload-rules
   ```

3. **Log out and back in** (no reboot needed on Jetson)

---

## Verifying Serial Port

### Check Device Exists

```bash
# Raspberry Pi
ls -l /dev/ttyAMA0

# Jetson Nano
ls -l /dev/ttyTHS1
```

**Expected output:**
```
crw-rw---- 1 root dialout 204, 64 Jan  1 12:00 /dev/ttyAMA0
```

### Check Permissions

```bash
# Check your groups
groups

# Should include: dialout
```

**If `dialout` is missing:**
```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### Check UART Configuration

```bash
# Raspberry Pi
stty -F /dev/ttyAMA0

# Jetson Nano
stty -F /dev/ttyTHS1
```

**Expected:** Should show `speed 115200 baud`

### Set Baud Rate

```bash
# Raspberry Pi
sudo stty -F /dev/ttyAMA0 115200 raw -echo

# Jetson Nano
sudo stty -F /dev/ttyTHS1 115200 raw -echo
```

---

## Testing Serial Communication

### Using Test Script

```bash
cd ~/hoverbot/platforms/common/scripts

# Raspberry Pi
python3 test_serial.py --port /dev/ttyAMA0

# Jetson Nano
python3 test_serial.py --port /dev/ttyTHS1
```

### Using ROS 2 Driver

```bash
# Raspberry Pi 4
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=$HOME/hoverbot/platforms/raspberry-pi4/config/hoverbot_driver.yaml

# Raspberry Pi 5
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=$HOME/hoverbot/platforms/raspberry-pi5/config/hoverbot_driver.yaml

# Jetson Nano
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=$HOME/hoverbot/platforms/jetson-nano/config/hoverbot_driver.yaml
```

---

## Troubleshooting

### "Permission denied" on Serial Port

**Problem:** Can't open `/dev/ttyAMA0` or `/dev/ttyTHS1`

**Solution:**
```bash
# Add yourself to dialout group
sudo usermod -aG dialout $USER

# IMPORTANT: Log out and back in (or reboot)
sudo reboot
```

### "No such file or directory"

**Problem:** Serial device doesn't exist

**Raspberry Pi:**
```bash
# Check boot config
sudo cat /boot/firmware/config.txt | grep uart

# Should see: enable_uart=1
# If not, add it and reboot
```

**Jetson Nano:**
```bash
# Check if UART is enabled
cat /proc/device-tree/serial@70006040/status

# Should show: "okay"
```

### "Input/output error"

**Problem:** Serial port configuration issue

**Solution:**
```bash
# Raspberry Pi
sudo stty -F /dev/ttyAMA0 115200 raw -echo
sudo systemctl stop serial-getty@ttyAMA0.service

# Jetson Nano
sudo stty -F /dev/ttyTHS1 115200 raw -echo
sudo systemctl stop nvgetty
```

### Serial Console Interference

**Raspberry Pi:**
```bash
# Disable serial console permanently
sudo systemctl disable serial-getty@ttyAMA0.service
```

**Jetson Nano:**
```bash
# Disable nvgetty
sudo systemctl disable nvgetty
```

### Wrong Baud Rate

**Problem:** Communication fails or garbled data

**Check current baud rate:**
```bash
stty -F /dev/ttyAMA0 | grep speed  # or /dev/ttyTHS1
```

**Set correct baud rate:**
```bash
sudo stty -F /dev/ttyAMA0 115200 raw -echo  # or /dev/ttyTHS1
```

**Verify firmware baud rate:** Must be 115200 (see `firmware/config/config.h`)

---

## Protocol Specification

### Common to All Platforms

All platforms use the same serial protocol:

| Parameter | Value |
|-----------|-------|
| **Baud Rate** | 115200 |
| **Data Bits** | 8 |
| **Parity** | None |
| **Stop Bits** | 1 |
| **Flow Control** | None |
| **Mode** | Raw (no echo, no processing) |

### Packet Structure

```
Byte 0-1: Start frame (0xABCD, little-endian)
Byte 2-3: Steer value (-1000 to +1000, int16 little-endian)
Byte 4-5: Speed value (-1000 to +1000, int16 little-endian)
Byte 6-7: Checksum (start XOR steer XOR speed, uint16 little-endian)
```

**This protocol is identical across all platforms!** Only the device path changes.

---

## Override Serial Port at Runtime

### Command Line Override

```bash
# Override via launch argument
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  serial_port:=/dev/ttyTHS1
```

### Python Code Override

```python
# In your launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hoverbot_driver',
            executable='hoverbot_driver_node',
            parameters=[{
                'serial_port': '/dev/ttyTHS1',  # Override here
                'baud_rate': 115200
            }]
        )
    ])
```

---

## Best Practices

### ✅ DO:

- Use platform-specific config files
- Test serial port before running ROS nodes
- Add yourself to `dialout` group
- Disable serial console
- Use raw mode with no echo
- Set baud rate to 115200

### ❌ DON'T:

- Hardcode serial ports in code
- Run as root to bypass permissions
- Use serial console and data UART simultaneously
- Change baud rate without updating firmware
- Forget to log out after adding to dialout group

---

## Quick Reference

```bash
# Check if serial device exists
ls -l /dev/ttyAMA0    # Pi4/Pi5
ls -l /dev/ttyTHS1    # Jetson

# Check permissions
groups | grep dialout

# Set baud rate
sudo stty -F /dev/ttyAMA0 115200 raw -echo  # Pi
sudo stty -F /dev/ttyTHS1 115200 raw -echo  # Jetson

# Test communication
cd ~/hoverbot/platforms/common/scripts
python3 test_serial.py --port /dev/ttyAMA0  # Or /dev/ttyTHS1
```

---

## See Also

- [Platform Setup Guide](PLATFORM_SETUP_GUIDE.md)
- [Hardware Wiring](../hardware/README.md)
- [Firmware Configuration](FIRMWARE.md)
- [Common Platform Utilities](../platforms/common/README.md)

---

**Questions?** Open an issue or discussion on GitHub!
