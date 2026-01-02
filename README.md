# HoverBot - Autonomous Indoor Mapping Robot

An autonomous mobile robot built on a hoverboard platform with ROS 2, RPLidar A1 for SLAM mapping, and Nav2 for autonomous navigation.

<<<<<<< HEAD
## 🎯 Project Goals

- Map indoor environments autonomously using SLAM
- Navigate mapped spaces avoiding obstacles
- Reuse hoverboard motors and battery for cost-effective platform
- Learn ROS 2, robotics software, and embedded systems
=======
---

## 🚀 Quick Start: Choose Your Platform

HoverBot supports multiple hardware platforms. **Choose yours to get started:**

| Platform | Status | Best For | Quick Link |
|----------|--------|----------|------------|
| 🥧 **[Raspberry Pi 5](platforms/raspberry-pi5/README.md)** | ✅ **Recommended** | Best performance & latest software | [Setup Guide →](platforms/raspberry-pi5/docs/SETUP.md) |
| 🥧 **[Raspberry Pi 4](platforms/raspberry-pi4/README.md)** | ✅ Supported | Budget-friendly, proven stability | [Setup Guide →](platforms/raspberry-pi4/README.md) |
| 🤖 **[Jetson Nano](platforms/jetson-nano/README.md)** | ⚠️ Experimental | AI/Vision applications | [Setup Guide →](platforms/jetson-nano/README.md) |

**Not sure which to choose?** See the **[Platform Comparison Guide](docs/PLATFORM_SETUP_GUIDE.md)**

---

## 🎯 What This Repository Contains

This repo has **everything you need** - complete working files for each component:

- **`platforms/`** - Platform-specific configurations (Pi4, Pi5, Jetson Nano)
- **`ros2_ws/`** - Complete ROS 2 workspace (platform-independent)
- **`firmware/`** - STM32 hoverboard controller configuration (platform-independent)
- **`hardware/`** - Wiring diagrams and hardware documentation
- **`docs/`** - Comprehensive documentation and guides

**Platform-specific files** are in `platforms/{platform}/` - **ROS 2 code is universal!**

---

## 📁 Repository Structure
>>>>>>> claude/change-branch-ZGp4e

## 🤖 System Overview
```
<<<<<<< HEAD
                    ┌─────────────────┐
                    │  Raspberry Pi 4 │
                    │  ROS 2 Humble   │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
        ┌─────▼────┐   ┌────▼─────┐  ┌────▼────┐
        │RPLidar A1│   │Hoverboard│  │ Network │
        │  (USB)   │   │  (UART)  │  │  (WiFi) │
        └──────────┘   └──────────┘  └─────────┘
             │               │              │
        360° Lidar    Differential      Dev Machine
        Scanning      Drive Motors      (RViz/Control)
```

## ✅ Current Status (Dec 31, 2024)

**Software:** ✅ Complete and validated (7 sensors integrated)
- Driver communicating with hoverboard (99.3% success rate)
- BNO055 IMU calibrated (9.80 m/s² gravity - perfect!)
- Sensor fusion (EKF) fusing odometry + IMU at 50 Hz
- RPLidar scanning and publishing data at 7.6 Hz
- RealSense D435 publishing depth at 15 Hz
- ELP USB camera publishing RGB at 30 Hz
- SLAM Toolbox creating maps at 0.1 Hz
- Nav2 configured for autonomous navigation
- Single-command V3 launch system
- RViz real-time visualization working
- Persistent device names (udev rules)

**Hardware:** ⏳ Assembly pending
- Components tested individually
- Bench testing successful
- Awaiting physical robot construction

## 🛠️ Hardware

| Component | Model | Function | Status |
|-----------|-------|----------|--------|
| Computer | Raspberry Pi 4 (4GB) | Main controller | ✅ Working |
| Motors | Hoverboard (EFeru firmware) | Differential drive | ✅ Working |
| Lidar | RPLidar A1 | 2D SLAM mapping | ✅ Working |
| IMU | Adafruit BNO055 | Orientation, sensor fusion | ✅ Working |
| Depth Camera | Intel RealSense D435 | 3D obstacles (15 Hz) | ✅ Working |
| RGB Camera | ELP USB GS1200P01 | Visual tasks (30 Hz) | ✅ Working |
| Platform | Hoverboard deck | Robot chassis | ⏳ Assembly needed |
| Battery | Hoverboard 36V | Power supply | ✅ Working |

### Wiring
- **UART:** Pi GPIO14/15 (TX/RX) ↔ Hoverboard USART3 (115200 baud)
- **Power:** Hoverboard battery → Pi via buck converter
- **Lidar:** USB connection to Pi

### Additional Sensors
- **IMU:** BNO055 via I2C (address 0x28)
- **Depth Camera:** RealSense D435 via USB
- **RGB Camera:** ELP GS1200P01 via USB (persistent /dev/elp_camera)

See `hardware/` for detailed wiring diagrams.

## 💻 Software Stack

- **OS:** Ubuntu 22.04 LTS Server (ARM64)
- **ROS:** ROS 2 Humble Hawksbill
- **SLAM:** SLAM Toolbox (Karto SLAM)
- **Navigation:** Nav2 (Navigation2)
- **Visualization:** RViz2
- **Automation:** Tmux session management

### ROS 2 Packages
```
hoverbot/ros2_ws/src/
├── hoverbot_driver      # Serial communication with hoverboard
├── hoverbot_bringup     # Launch files and configurations
└── hoverbot_description # Robot URDF (future)
```
=======
hoverbot/
│
├── platforms/                     # 🎯 PLATFORM-SPECIFIC FILES
│   ├── common/                    # Shared utilities for all platforms
│   │   ├── scripts/              # Generic test scripts
│   │   ├── config/               # Config templates
│   │   └── README.md
│   │
│   ├── raspberry-pi5/             # ✅ Raspberry Pi 5 (Recommended)
│   │   ├── config/
│   │   │   └── hoverbot_driver.yaml  # Pi5 serial port config
│   │   ├── scripts/
│   │   │   ├── setup_uart.sh     # Pi5 UART setup
│   │   │   └── test_serial.py    # Serial tester
│   │   ├── docs/
│   │   │   └── SETUP.md          # Complete Pi5 setup guide
│   │   └── README.md             # Pi5 overview
│   │
│   ├── raspberry-pi4/             # ✅ Raspberry Pi 4 (Supported)
│   │   ├── config/
│   │   │   └── hoverbot_driver.yaml  # Pi4 serial port config
│   │   ├── scripts/
│   │   │   └── setup_uart.sh     # Pi4 UART setup
│   │   ├── docs/
│   │   │   └── SETUP.md          # Complete Pi4 setup guide
│   │   └── README.md             # Pi4 overview
│   │
│   └── jetson-nano/               # ⚠️ Jetson Nano (Experimental)
│       ├── config/
│       │   └── hoverbot_driver.yaml  # Jetson serial port config (/dev/ttyTHS1)
│       ├── scripts/
│       │   └── setup_uart.sh     # Jetson UART setup
│       ├── docs/
│       │   └── SETUP.md          # Jetson setup guide (TODO)
│       └── README.md             # Jetson overview
│
├── ros2_ws/                       # 🤖 ROS 2 WORKSPACE (Platform-Independent!)
│   └── src/
│       ├── hoverbot_description/ # Robot URDF model & Gazebo simulation
│       ├── hoverbot_driver/      # Serial driver node (works on all platforms)
│       └── hoverbot_bringup/     # Launch files
│
├── firmware/                      # 🔧 STM32 HOVERBOARD FIRMWARE (Platform-Independent!)
│   ├── config/
│   │   └── config.h              # Complete working config.h
│   ├── bin/
│   │   └── firmware.bin          # Pre-compiled binary (ready to flash)
│   └── README.md
│
├── hardware/                      # 📐 HARDWARE DOCUMENTATION
│   ├── wiring/                   # Connection diagrams
│   ├── schematics/               # Pinouts and schematics
│   └── README.md
│
├── docs/                          # 📚 DOCUMENTATION
│   ├── PLATFORM_SETUP_GUIDE.md   # Platform comparison & selection
│   ├── SERIAL_PORT_GUIDE.md      # Cross-platform serial config
│   ├── FIRMWARE.md               # Firmware configuration
│   └── JOURNAL.md                # Development log
│
└── README.md                      # This file
```

**Key Insight:** Only `platforms/` directory changes between hardware. ROS 2 code is universal!

---
>>>>>>> claude/change-branch-ZGp4e

## 🚀 Quick Start

### Prerequisites
- Raspberry Pi 4 running Ubuntu 22.04 + ROS 2 Humble
- RPLidar A1 connected via USB
- Hoverboard with EFeru firmware via UART
- Dev machine with ROS 2 Humble (for visualization)

### One-Command Startup
```bash
# On dev machine
cd ~/hoverbot/scripts
./hoverbot_startup.sh

# Reattach anytime
tmux attach -t hoverbot

# Stop everything
tmux kill-session -t hoverbot
```

This launches all components automatically:
1. Hoverboard driver (odometry + motor control)
2. Static transforms (base_link → laser)
3. RPLidar node (lidar scanning)
4. SLAM Toolbox (mapping)
5. Teleop window (keyboard control)

### Visualization
```bash
# On dev machine (after robot is running)
rviz2

# Add displays: TF, LaserScan, Map, Odometry
# Watch real-time mapping!
```

### Save a Map
```bash
# After driving around and building a map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

## 📚 Documentation

All documentation in `docs/pi4/`:

- **[PROJECT_STATUS.md](docs/pi4/PROJECT_STATUS.md)** - Complete project overview and current status
- **[QUICK_START.md](docs/pi4/QUICK_START.md)** - Copy-paste startup commands
- **[SLAM_TESTING_RESULTS.md](docs/pi4/SLAM_TESTING_RESULTS.md)** - Detailed test results
- **[RVIZ_SETUP.md](docs/pi4/RVIZ_SETUP.md)** - Visualization setup guide
- **[NAV2_SETUP.md](docs/pi4/NAV2_SETUP.md)** - Autonomous navigation guide

## 🔧 Development Setup

### Clone Repository
```bash
git clone https://github.com/Dasovon/hoverbot.git
cd hoverbot
git checkout pi4  # Pi 4 working branch
```

### Build on Raspberry Pi
```bash
cd ~/hoverbot/ros2_ws
colcon build
source install/setup.bash
```

### Development Machine Setup
```bash
# Install ROS 2 Humble
sudo apt install ros-humble-desktop

# Install RViz
sudo apt install ros-humble-rviz2

# Configure for robot communication
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 📊 Performance Metrics

| Metric | Target | Actual | Status |
|--------|--------|--------|--------|
| Serial Success Rate | >95% | 99.3% | ✅ |
| Odometry Rate | 50 Hz | 50 Hz | ✅ |
| Lidar Scan Rate | 10 Hz | 7 Hz | ⚠️ |
| SLAM Map Rate | 0.1 Hz | 0.1 Hz | ✅ |

## 🎓 Learning Resources

This project covers:
- ROS 2 ecosystem (nodes, topics, services, actions)
- SLAM algorithms (Karto SLAM, loop closure)
- Path planning (NavFn, DWB controller)
- Differential drive kinematics
- Sensor fusion (lidar + odometry)
- Embedded systems (UART communication, firmware)
- Robot visualization (RViz, TF frames)

## 🐛 Known Issues

1. **RPLidar Buffer Overflow** - Workaround via sequential launch with delays (tmux automation)
2. **SLAM Initial Dropping** - Messages drop for ~5s during startup (self-resolves)
3. **Map Save Timing** - Must time map_saver with 10s publish cycle

See full issue list in `docs/pi4/PROJECT_STATUS.md`

## 🗺️ Roadmap

### Phase 1: Software (✅ Complete)
- [x] Hoverboard serial driver
- [x] RPLidar integration
- [x] SLAM mapping
- [x] Tmux automation
- [x] RViz visualization
- [x] Nav2 configuration

### Phase 2: Hardware Assembly (⏳ Next)
- [ ] Mount RPLidar on chassis
- [ ] Secure wiring and components
- [ ] Verify mechanical clearances
- [ ] Add emergency stop button

### Phase 3: Testing & Tuning
- [ ] SLAM mapping in real environment
- [ ] Odometry calibration
- [ ] Nav2 parameter tuning
- [ ] Obstacle avoidance testing

### Phase 4: Advanced Features
- [ ] Visual odometry (camera)
- [ ] IMU integration
- [ ] Web interface
- [ ] Mobile app control

## 🤝 Contributing

This is a personal learning project, but suggestions and ideas are welcome! Open an issue or submit a PR.

## 📄 License

MIT License - See LICENSE file

## 🙏 Acknowledgments

- [EFeru Hoverboard Firmware](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC) - Motor control firmware
- [SLAMTEC](https://www.slamtec.com/) - RPLidar A1
- [ROS 2](https://docs.ros.org/) - Robot Operating System
- [Nav2](https://navigation.ros.org/) - Navigation framework
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) - SLAM implementation

## 📞 Contact

- GitHub: [@Dasovon](https://github.com/Dasovon)
- Repository: [github.com/Dasovon/hoverbot](https://github.com/Dasovon/hoverbot)

---

**Built with ❤️ and lots of learning**

*Last updated: December 31, 2024*4*
