# Raspberry Pi Configuration - Quick Start

**Everything you need to set up the Raspberry Pi for hoverboard control.**

---

## 📁 What's in This Folder

```
raspberry-pi/
├── scripts/
│   ├── setup_uart.sh         # Automated UART setup
│   ├── test_serial.py        # Complete serial test suite
│   └── hoverboard_control.py # Motor control library
├── config/
│   ├── config.txt            # Boot configuration
│   └── README.md             # Config file documentation  
├── docs/
│   ├── SETUP.md              # Detailed setup guide
│   ├── UART.md               # UART configuration details
│   └── TROUBLESHOOTING.md    # Common issues
└── README.md                 # This file
```

---

## 🚀 Quick Setup (10 minutes)

### 1. Run Setup Script

```bash
cd raspberry-pi/scripts/
chmod +x setup_uart.sh
./setup_uart.sh
```

This will:
- ✅ Enable UART
- ✅ Disable Bluetooth
- ✅ Disable serial console
- ✅ Add you to dialout group
- ✅ Configure baud rate

### 2. Reboot

```bash
sudo reboot
```

### 3. Test Communication

```bash
# After reboot, log out and back in
cd raspberry-pi/scripts/
python3 test_serial.py
```

**Expected:** Beeping stops, motors respond to commands

---

## 🔌 Hardware Connections

Connect Pi to **right sideboard** connector:

```
Raspberry Pi              Hoverboard
Pin 6 (GND)        →      Pin 2 (GND)
Pin 8 (GPIO14 TX)  →      Pin 4 (RX)
Pin 10 (GPIO15 RX) →      Pin 3 (TX)
```

**Critical:** Double-check TX→RX and RX→TX crossover!

---

## 🐍 Python Scripts

### test_serial.py
Complete test suite:
- Connection test
- Motor movement test
- Telemetry reading
- Interactive keyboard control

```bash
python3 scripts/test_serial.py
```

### hoverboard_control.py
Reusable control library:
```python
from hoverboard_control import HoverboardController

controller = HoverboardController()
controller.connect()
controller.send_command(steer=100, speed=100)  # Forward
controller.disconnect()
```

---

## ⚙️ Configuration Files

### config.txt
Boot configuration for UART:
```
enable_uart=1
dtoverlay=disable-bt
```

Copy to `/boot/firmware/config.txt` or use setup script.

---

## ✅ Verification Checklist

After setup:

- [ ] UART device exists: `ls -l /dev/ttyAMA0`
- [ ] Baud rate correct: `stty -F /dev/ttyAMA0 | grep 115200`
- [ ] In dialout group: `groups | grep dialout`
- [ ] Beeping stops when sending commands
- [ ] Motors respond to test script

---

## 🛠️ Troubleshooting

### Permission denied on /dev/ttyAMA0
```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### UART not found
```bash
# Check boot config
cat /boot/firmware/config.txt | grep uart

# Should show: enable_uart=1
```

### Beeping won't stop
- Check wiring (TX/RX crossover)
- Verify baud rate: `stty -F /dev/ttyAMA0 115200 raw -echo`
- Confirm firmware flashed correctly

### Import errors
```bash
pip3 install pyserial
```

---

## 📚 Documentation

- **`docs/SETUP.md`** - Complete Pi setup guide
- **`docs/UART.md`** - UART technical details
- **`docs/TROUBLESHOOTING.md`** - Common issues and fixes

---

## 🔗 Next Steps

1. **Test serial communication** - `python3 scripts/test_serial.py`
2. **Build ROS 2 workspace** - See `../ros2_ws/README.md`
3. **Integrate sensors** - Add RPLidar

---

**Pi setup complete! Test communication before moving to ROS 2.**
