# HoverBot - Autonomous Indoor Mapping Robot

Complete, working configurations for building an autonomous robot from a hoverboard platform.

## 🎯 What This Repository Contains

This repo has **everything you need** - complete working files for each component:

- **`firmware/`** - Complete STM32 hoverboard controller configuration
- **`raspberry-pi/`** - Complete Raspberry Pi setup and control scripts
- **`ros2_ws/`** - Complete ROS 2 workspace with all packages
- **`hardware/`** - Wiring diagrams and hardware documentation

Each folder is **self-contained** with everything needed to make that component work.

---

## 📁 Repository Structure

```
hoverbot/
│
├── firmware/                      # STM32 Hoverboard Controller
│   ├── config/
│   │   └── config.h              # Complete working config.h
│   ├── bin/
│   │   └── firmware.bin          # Pre-compiled binary (ready to flash)
│   ├── docs/
│   │   ├── FIRMWARE.md           # Detailed configuration guide
│   │   ├── FLASHING.md           # Step-by-step flashing guide
│   │   └── PROTOCOL.md           # Serial protocol specification
│   └── README.md                 # Firmware quick start
│
├── raspberry-pi/                  # Raspberry Pi Configuration
│   ├── scripts/
│   │   ├── hoverboard_control.py # Complete motor control script
│   │   ├── test_serial.py        # Serial communication test
│   │   └── setup_uart.sh         # UART setup script
│   ├── config/
│   │   ├── config.txt            # Boot configuration
│   │   ├── cmdline.txt           # Boot command line
│   │   └── udev-rules/           # USB device rules
│   ├── docs/
│   │   ├── SETUP.md              # Complete Pi setup guide
│   │   ├── UART.md               # UART configuration details
│   │   └── TROUBLESHOOTING.md    # Common issues and fixes
│   └── README.md                 # Pi quick start
│
├── ros2_ws/                       # ROS 2 Workspace
│   ├── src/
│   │   ├── hoverbot_description/ # Robot URDF model
│   │   ├── hoverbot_driver/      # Serial driver node
│   │   ├── hoverbot_bringup/     # Launch files
│   │   └── hoverbot_navigation/  # Navigation configs
│   ├── docs/
│   │   ├── ROS2_SETUP.md         # ROS 2 installation
│   │   ├── PACKAGES.md           # Package documentation
│   │   └── USAGE.md              # How to use the robot
│   └── README.md                 # ROS 2 quick start
│
├── hardware/                      # Hardware Documentation
│   ├── wiring/
│   │   ├── uart_wiring.md        # UART connection guide
│   │   ├── power_wiring.md       # Power distribution
│   │   └── sensor_wiring.md      # Sensor connections
│   ├── schematics/
│   │   └── pinouts.md            # All connector pinouts
│   ├── docs/
│   │   ├── ASSEMBLY.md           # Hardware assembly guide
│   │   └── BOM.md                # Bill of materials
│   └── README.md                 # Hardware overview
│
├── docs/                          # General Documentation
│   ├── QUICKSTART.md             # Get started in 10 minutes
│   ├── JOURNAL.md                # Development log
│   └── FAQ.md                    # Frequently asked questions
│
└── README.md                      # This file
```

---

## 🚀 Quick Start

### Option 1: Flash and Go (10 minutes)

If you just want to get it working:

1. **Flash firmware:**
   ```bash
   cd firmware/
   # Follow firmware/README.md to flash bin/firmware.bin
   ```

2. **Setup Raspberry Pi:**
   ```bash
   cd raspberry-pi/
   ./scripts/setup_uart.sh
   ```

3. **Test it:**
   ```bash
   python3 raspberry-pi/scripts/test_serial.py
   ```

### Option 2: Full Build (1-2 hours)

For complete setup with ROS 2:

1. Flash firmware (see `firmware/README.md`)
2. Configure Raspberry Pi (see `raspberry-pi/README.md`)
3. Build ROS 2 workspace (see `ros2_ws/README.md`)
4. Wire hardware (see `hardware/README.md`)

See `docs/QUICKSTART.md` for detailed walkthrough.

---

## 🔧 Each Component is Self-Contained

### Firmware Folder
- **Complete `config.h`** with all working settings
- **Pre-compiled `.bin`** ready to flash
- **Detailed docs** explaining every setting
- **No need to clone** the entire EFeru repo

### Raspberry Pi Folder
- **Working Python scripts** for motor control
- **Complete config files** for boot and UART
- **Setup scripts** to automate configuration
- **Everything needed** for Pi to communicate with hoverboard

### ROS 2 Workspace
- **All packages** ready to build
- **Launch files** pre-configured
- **URDF model** complete
- **Just `colcon build`** and run

### Hardware Folder
- **Wiring diagrams** with photos/drawings
- **Pin mappings** for all connectors
- **Assembly instructions** step-by-step
- **BOM** with part numbers and links

---

## 📋 What's Working

### ✅ Firmware
- Power button latches correctly (BOARD_VARIANT 1)
- USART3 serial communication at 115200 baud
- Tank steering for differential drive
- Continuous feedback telemetry
- Safety timeout with beeping

### ✅ Raspberry Pi
- UART configured and operational
- Python control scripts validated
- Motor commands working (forward/backward/turn)
- Telemetry reading (battery, temp, speed)

### ✅ Hardware
- All wiring documented and tested
- UART connection to right sideboard (5V tolerant)
- Power distribution safe and stable
- RPLidar A1 mounted and ready

### ✅ ROS 2
- URDF robot model complete
- Packages building cleanly
- Launch files operational
- Ready for driver integration

---

## 🎓 Documentation Philosophy

Each folder has:

1. **README.md** - Quick start for that component
2. **docs/** - Detailed guides and troubleshooting
3. **Working files** - No placeholders, everything functional

You can work on **one component at a time** without dependencies on others.

---

## 🔨 Hardware Requirements

- **Hoverboard:** Single mainboard with STM32F103
- **Computer:** Raspberry Pi 5 (4GB+)
- **Sensor:** RPLidar A1
- **Battery:** 36V 10S Li-ion
- **Programmer:** ST-Link V2
- **Cables:** Dupont wires for UART

Full BOM in `hardware/docs/BOM.md`

---

## 💡 Key Technical Details

### Serial Protocol
- **Packet:** 8 bytes (start, steer, speed, checksum)
- **Checksum:** `start XOR steer XOR speed`
- **Baud:** 115200
- **Rate:** 50 Hz

### Firmware Config
- **BOARD_VARIANT:** 1 (critical!)
- **TANK_STEERING:** Enabled
- **USART3:** Right sideboard
- **Timeout:** 800ms

### Pi GPIO
- **TX:** GPIO14 (Pin 8) → Hoverboard RX
- **RX:** GPIO15 (Pin 10) → Hoverboard TX
- **GND:** Pin 6 → Hoverboard GND

---

## 🛠️ Development Status

| Component | Status | Notes |
|-----------|--------|-------|
| Firmware | ✅ Complete | Validated and working |
| Pi Scripts | ✅ Complete | Motor control functional |
| Hardware | ✅ Complete | All connections tested |
| ROS 2 URDF | ✅ Complete | Robot model ready |
| ROS 2 Driver | 🔄 In Progress | Serial bridge needed |
| SLAM | 📋 Planned | slam_toolbox config |
| Navigation | 📋 Planned | Nav2 integration |

---

## 📖 Usage Examples

### Test Motor Control
```bash
cd raspberry-pi/scripts/
python3 test_serial.py
```

### Launch ROS 2 Visualization
```bash
cd ros2_ws/
source install/setup.bash
ros2 launch hoverbot_bringup state_publisher.launch.py
```

### Flash New Firmware
```bash
cd firmware/
# Edit config/config.h if needed
# Flash bin/firmware.bin using ST-Link
```

---

## 🆘 Getting Help

1. Check the **component README** in each folder
2. Look in **docs/** subfolder for detailed guides
3. Read **docs/FAQ.md** for common questions
4. Check **docs/JOURNAL.md** for issues we've solved
5. Open an issue on GitHub

---

## 🤝 Contributing

Each component folder is independent. You can improve:
- Firmware configurations
- Python control scripts
- ROS 2 packages
- Documentation

See individual README files for component-specific guidelines.

---

## 📜 License

- **Firmware:** GPL-3.0 (EFeru FOC firmware)
- **Scripts & ROS packages:** MIT
- **Documentation:** CC-BY-4.0

---

## 🙏 Acknowledgments

- **EFeru** - Hoverboard FOC firmware
- **ROS 2 Community** - Robotics framework
- **Raspberry Pi Foundation** - Single-board computer

---

## 🗺️ Project Roadmap

- [x] Firmware configuration and flashing
- [x] Serial communication protocol
- [x] Raspberry Pi control scripts
- [x] Hardware assembly and wiring
- [x] ROS 2 workspace structure
- [ ] ROS 2 serial driver node
- [ ] RPLidar integration
- [ ] SLAM mapping
- [ ] Autonomous navigation

---

**Each folder is self-contained and ready to use!**

Pick the component you want to work on and dive in. Everything you need is right there.

---

**Last Updated:** December 2024  
**Status:** All components operational, ready for integration
