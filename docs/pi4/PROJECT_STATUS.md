# HoverBot Project - Current Status (Dec 27, 2025)

## Quick Context for New Chat Sessions

**Project:** Autonomous indoor mapping robot using hoverboard platform
**GitHub:** https://github.com/Dasovon/hoverbot (pi4 branch)
**Status:** Software fully validated, ready for startup automation

## Hardware Setup
- **Dev Machine:** Ubuntu 22.04 + ROS 2 Humble (x86_64)
- **Robot Computer:** Raspberry Pi 4, Ubuntu 22.04 + Humble, IP: 192.168.86.20, hostname: hoverbot
- **Motor Controller:** Hoverboard with EFeru firmware, UART /dev/ttyAMA0, 115200 baud
- **Lidar:** RPLidar A1, USB /dev/ttyUSB0
- **Not Assembled:** RPLidar and hoverboard are separate (bench testing only)

## Working Components ✅
1. Driver: 99.3% serial success, 50Hz odometry, TF publishing
2. RPLidar: Scanning at 7Hz, health OK
3. SLAM: Maps building, 0.1Hz map publishing
4. Teleop: Keyboard control working
5. All Git commits pushed to pi4 branch

## Current Startup (5 Terminals Required)
See `docs/pi4/QUICK_START.md` for copy-paste commands

## Known Issues
- RPLidar buffer overflow in combined launch files → Must launch separately
- Initial SLAM message dropping → Self-resolves after startup

## Repository Structure
```
hoverbot/
├── firmware/          # Hoverboard config (BOARD_VARIANT=1, USART3)
├── raspberry-pi/      # Test scripts
├── ros2_ws/
│   └── src/
│       ├── hoverbot_driver/      # Serial driver package ✅
│       ├── hoverbot_bringup/     # Launch files
│       └── hoverbot_description/ # URDF (incomplete)
├── hardware/          # Wiring docs
└── docs/pi4/          # Pi 4 specific docs

main branch    → Pi 5 ready (Ubuntu 24.04 + Jazzy)
pi4 branch     → Pi 4 working (Ubuntu 22.04 + Humble) ← ACTIVE
```

## Next Tasks (Priority Order)
1. ⏭️ Startup automation (tmux/screen scripts)
2. ⏭️ RViz visualization from dev machine
3. ⏭️ Nav2 configuration
4. ⏭️ Parameter tuning
5. ⏭️ Fix RPLidar buffer overflow
6. ⏭️ Physical assembly (when ready)

## Important Files
- Driver code: `ros2_ws/src/hoverbot_driver/hoverbot_driver/`
- SLAM config: `ros2_ws/src/hoverbot_bringup/config/slam_toolbox_params.yaml`
- Startup docs: `docs/pi4/QUICK_START.md`, `SLAM_TESTING_RESULTS.md`

## Copy This to Start New Chat
"I'm working on HoverBot - an autonomous mapping robot. Full context in my repo: https://github.com/Dasovon/hoverbot (pi4 branch). Current status: All ROS 2 software validated (driver, RPLidar, SLAM working). Hardware not assembled yet (bench testing). 

Current setup: Pi 4 (192.168.86.20) running Ubuntu 22.04 + Humble, 5-terminal startup sequence working. 

Next goal: Create tmux/screen startup automation to replace 5-terminal manual launch. See docs/pi4/PROJECT_STATUS.md for details."
