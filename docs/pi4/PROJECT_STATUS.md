# HoverBot Project - Current Status (Dec 27, 2025)

## Quick Context for New Chat Sessions

**Project:** Autonomous indoor mapping robot using hoverboard platform
**GitHub:** https://github.com/Dasovon/hoverbot (pi4 branch)
**Status:** Complete software stack validated and ready for hardware assembly

## Hardware Setup
- **Dev Machine:** Ubuntu 22.04 + ROS 2 Humble (x86_64)
- **Robot Computer:** Raspberry Pi 4, Ubuntu 22.04 + Humble, IP: 192.168.86.20, hostname: hoverbot
- **Motor Controller:** Hoverboard with EFeru firmware, UART /dev/ttyAMA0, 115200 baud, 99.3% success rate
- **Lidar:** RPLidar A1, USB /dev/ttyUSB0, 7Hz scan rate
- **Assembly Status:** ⚠️ Not assembled - RPLidar and hoverboard are separate (bench testing only)

## System Architecture
```
┌─────────────────────────────────────────────────────────────┐
│                     Dev Machine (Ubuntu 22.04)              │
│  - RViz2 (visualization)                                    │
│  - Tmux automation scripts                                  │
│  - Git repository                                           │
│  - ROS_DOMAIN_ID=0                                          │
└────────────────────────┬────────────────────────────────────┘
                         │ SSH + ROS 2 Network
                         │ (192.168.86.x)
┌────────────────────────┴────────────────────────────────────┐
│              Raspberry Pi 4 (192.168.86.20)                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ Tmux Session "hoverbot" (4 panes + teleop window)    │  │
│  │                                                       │  │
│  │  ┌──────────┬──────────┐                            │  │
│  │  │  DRIVER  │ TF_STATIC│  Window 0: Robot Stack     │  │
│  │  ├──────────┼──────────┤                            │  │
│  │  │ RPLIDAR  │   SLAM   │                            │  │
│  │  └──────────┴──────────┘                            │  │
│  │                                                       │  │
│  │  Window 1: Teleop (keyboard control)                 │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  Published Topics:                                          │
│    /odom (50 Hz) - Wheel odometry                          │
│    /scan (7 Hz) - RPLidar data                             │
│    /map (0.1 Hz) - SLAM map                                │
│    /tf, /tf_static - Coordinate transforms                 │
│    /cmd_vel - Velocity commands (from teleop)              │
└─────────────────────────────────────────────────────────────┘
```

## Working Components ✅

### 1. Driver Node (hoverbot_driver)
- **Status:** ✅ Fully functional
- **Serial Communication:** 99.3% success rate (146/147 packets)
- **Odometry:** Publishing at 50 Hz
- **TF Broadcasting:** odom → base_link transform
- **Safety:** cmd_vel timeout (0.5s) with auto-stop
- **Speed Limits:** 
  - Max linear: 2.59 m/s (300 RPM)
  - Max angular: 12.96 rad/s
- **Battery Monitoring:** 40.84V full charge, 24.3°C operating temp

### 2. RPLidar A1
- **Status:** ✅ Working
- **Health:** OK (firmware 1.29, hardware rev 7)
- **Scan Rate:** 7 Hz (target 10 Hz)
- **Sample Rate:** 8 KHz
- **Max Range:** 12m
- **Topic:** /scan publishing LaserScan messages

### 3. Static Transform Publisher
- **Status:** ✅ Working
- **Transform:** base_link → laser
- **Position:** 0.1m forward, 0.1m up from base_link
- **Publishing:** /tf_static at 10000 Hz

### 4. SLAM Toolbox
- **Status:** ✅ Mapping functional
- **Solver:** CeresSolver with SCHUR_JACOBI preconditioner
- **Map Publishing:** 0.1 Hz (every 10 seconds)
- **Resolution:** 0.05m per pixel
- **Loop Closure:** Enabled
- **Localization:** Functioning in mapped areas
- **Known Issue:** Initial message dropping during startup (self-resolves)

### 5. Tmux Automation
- **Status:** ✅ Production ready
- **Script:** `~/hoverbot/scripts/hoverbot_startup.sh`
- **Features:**
  - One-command startup from dev machine
  - Sequential component launch with proper delays
  - 2x2 pane layout + teleop window
  - Attach/detach capability
  - Session persistence
- **Usage:** `./hoverbot_startup.sh` → `tmux attach -t hoverbot`

### 6. RViz2 Visualization
- **Status:** ✅ Real-time visualization working
- **Displays:**
  - TF frames (coordinate axes)
  - LaserScan (live lidar data - updates instantly)
  - Map (SLAM-generated map)
  - Odometry (robot position)
- **Network:** ROS_DOMAIN_ID=0 configured on both machines
- **Performance:** Real-time updates confirmed (hand-moved lidar test)

### 7. Nav2 Configuration
- **Status:** ✅ Configured and ready for testing
- **Components:**
  - AMCL localization
  - DWB local planner (differential drive)
  - NavFn global planner
  - Behavior server (spin, backup, wait)
  - Costmap layers (static, obstacle, inflation)
  - Velocity smoother
- **Parameters:** Tuned for hoverbot (conservative speeds for safety)
- **Files:** 
  - `config/nav2/nav2_params.yaml`
  - `launch/nav2_bringup.launch.py`
- **Testing:** Blocked until hardware assembly

## Known Issues

### 1. RPLidar Buffer Overflow
- **Symptom:** Crashes when launched via combined launch files
- **Cause:** Known bug in rplidar_ros with ROS 2 Humble
- **Workaround:** Launch components separately with delays (tmux automation)
- **Impact:** Requires 5-component manual sequence (automated via tmux)
- **Status:** Workaround implemented and working

### 2. SLAM Initial Message Dropping
- **Symptom:** "Message Filter dropping message" during first 5-10 seconds
- **Cause:** Transform chain initialization timing
- **Impact:** None - self-resolves after startup
- **Status:** Expected behavior, no fix needed

### 3. Map Publishing Delay
- **Symptom:** map_saver_cli times out if not timed correctly
- **Cause:** Maps publish every 10 seconds
- **Workaround:** Run map_saver and wait for next publish cycle
- **Status:** Documented in usage guides

### 4. hoverbot_description Package Missing
- **Symptom:** Warning about missing local_setup.bash
- **Impact:** None - URDF/robot description not needed for current functionality
- **Status:** Commented out dependency, can add later for 3D visualization

## Repository Structure
```
hoverbot/
├── firmware/                    # Hoverboard firmware config
│   └── config.h                # BOARD_VARIANT=1, USART3
├── raspberry-pi/               # Test scripts and utilities
├── ros2_ws/
│   └── src/
│       ├── hoverbot_driver/    # ✅ Serial driver package
│       │   ├── hoverbot_driver/
│       │   │   └── driver_node.py (50Hz odometry, TF broadcast)
│       │   └── launch/
│       │       └── hoverbot_driver.launch.py
│       ├── hoverbot_bringup/   # ✅ Launch files and configs
│       │   ├── launch/
│       │   │   ├── robot_with_lidar.launch.py
│       │   │   ├── slam_mapping.launch.py
│       │   │   └── nav2_bringup.launch.py
│       │   └── config/
│       │       ├── slam_toolbox_params.yaml
│       │       └── nav2/
│       │           └── nav2_params.yaml
│       └── hoverbot_description/ # ⏳ URDF (not implemented)
├── scripts/                    # ✅ Automation scripts
│   └── hoverbot_startup.sh    # Tmux automation
├── hardware/                   # Wiring diagrams and schematics
└── docs/pi4/                   # ✅ Complete documentation
    ├── PROJECT_STATUS.md       # This file
    ├── README.md              # Pi 4 configuration overview
    ├── QUICK_START.md         # Copy-paste startup commands
    ├── SLAM_TESTING_RESULTS.md # Detailed test results
    ├── RVIZ_SETUP.md          # Visualization guide
    └── NAV2_SETUP.md          # Autonomous navigation guide

Branches:
  main   → Pi 5 target (Ubuntu 24.04 + ROS 2 Jazzy)
  pi4    → Pi 4 working (Ubuntu 22.04 + ROS 2 Humble) ← ACTIVE
```

## Performance Metrics

| Component | Metric | Target | Actual | Status |
|-----------|--------|--------|--------|--------|
| Serial Communication | Success Rate | >95% | 99.3% | ✅ Excellent |
| Odometry | Publish Rate | 50 Hz | 50 Hz | ✅ Perfect |
| RPLidar | Scan Rate | 10 Hz | 7 Hz | ⚠️ Lower but acceptable |
| SLAM Map | Publish Rate | 0.1 Hz | 0.1 Hz | ✅ Perfect |
| TF Broadcast | Rate | 50 Hz | 50 Hz | ✅ Perfect |
| Battery | Voltage | 36-42V | 40.84V | ✅ Good |
| Temperature | Operating Temp | <50°C | 24.3°C | ✅ Excellent |

## Startup Procedures

### Quick Start (Automated)
```bash
# 1. Power on: Hoverboard + RPLidar USB
# 2. On dev machine:
cd ~/hoverbot/scripts
./hoverbot_startup.sh

# 3. Reattach anytime:
tmux attach -t hoverbot

# 4. Stop everything:
tmux kill-session -t hoverbot
```

### Manual Startup (5 Terminals)
See `docs/pi4/QUICK_START.md` for detailed copy-paste commands.

### With RViz Visualization
```bash
# Terminal 1: Start robot
./hoverbot_startup.sh

# Terminal 2: Launch RViz (after robot running)
rviz2
# Add displays: TF, LaserScan, Map, Odometry
```

### With Nav2 (Future - Hardware Required)
```bash
# 1. Create map first (SLAM mapping session)
# 2. Launch Nav2 with saved map
ros2 launch hoverbot_bringup nav2_bringup.launch.py map:=/path/to/map.yaml
# 3. Set initial pose and goal in RViz
```

## Testing Status

### ✅ Validated (Bench Test)
- Serial communication and odometry
- RPLidar scanning
- SLAM map generation (static environment)
- Transform broadcasting (odom → base_link → laser)
- Teleop keyboard control
- RViz real-time visualization
- Tmux automation system
- Map saving/loading
- Nav2 configuration (launch only)

### ⏳ Pending (Hardware Assembly Required)
- Real SLAM mapping (robot movement)
- Autonomous navigation
- Dynamic obstacle avoidance
- Localization accuracy (AMCL)
- Path following performance
- Battery endurance testing
- Motor control under load

## Next Steps (Priority Order)

### Immediate (Software Complete)
1. ✅ ~~Tmux automation~~ **DONE**
2. ✅ ~~RViz visualization~~ **DONE**
3. ✅ ~~Nav2 configuration~~ **DONE**
4. ⏭️ Create comprehensive README for repository
5. ⏭️ Optional: Parameter tuning documentation
6. ⏭️ Optional: Investigate RPLidar buffer overflow fix

### Hardware Assembly Phase
1. ⏭️ Design/print mounting brackets for RPLidar
2. ⏭️ Mount RPLidar on hoverbot chassis
3. ⏭️ Secure all wiring (Pi, hoverboard, lidar)
4. ⏭️ Verify clearances for wheel movement
5. ⏭️ Add emergency stop button (recommended)
6. ⏭️ Battery management system (optional)

### Testing & Validation Phase
1. ⏭️ Test SLAM mapping in small controlled area
2. ⏭️ Verify odometry accuracy over distance
3. ⏭️ Tune Nav2 parameters for actual robot
4. ⏭️ Test obstacle avoidance
5. ⏭️ Create production maps of deployment environment
6. ⏭️ Endurance testing
7. ⏭️ Edge case handling (stuck recovery, etc.)

### Advanced Features (Future)
1. ⏭️ Visual odometry (camera integration)
2. ⏭️ IMU for better localization
3. ⏭️ Multi-floor mapping
4. ⏭️ Autonomous charging/docking
5. ⏭️ Web interface for monitoring
6. ⏭️ Mobile app control

## Key Learnings

### Firmware Configuration
- BOARD_VARIANT=1 critical for power latch and GPIO mapping
- VARIANT_USART required for serial control
- USART3 on right sideboard for 5V tolerance
- Serial protocol: 8-byte packets at 115200 baud with 0xABCD start frame

### ROS 2 Integration
- Timing is critical for component startup (RPLidar buffer overflow)
- Transform chain must be complete before SLAM starts
- ROS_DOMAIN_ID must match across all machines
- Odometry must publish continuously for Nav2 to work

### SLAM Mapping
- Requires robot motion for accurate mapping
- Loop closure improves map consistency
- Resolution vs map size tradeoff (0.05m chosen)
- Static lidar scans don't update map (need odometry changes)

### Network Communication
- ROS 2 DDS discovery works on same subnet
- Sourcing ROS in .bashrc essential for consistency
- Topics visible but data not flowing = domain mismatch
- Real-time visualization requires proper domain configuration

## Documentation Catalog

All documentation in `docs/pi4/`:

1. **PROJECT_STATUS.md** (this file) - Complete project overview
2. **README.md** - Pi 4 hardware and software configuration
3. **QUICK_START.md** - Fast reference for startup commands
4. **SLAM_TESTING_RESULTS.md** - Detailed testing results and metrics
5. **RVIZ_SETUP.md** - Visualization setup and usage
6. **NAV2_SETUP.md** - Autonomous navigation guide

## Copy This to Start New Chat
```
I'm working on HoverBot - an autonomous mapping robot using a hoverboard platform. 

**Current Status:**
- All software validated and working (driver, RPLidar, SLAM, Nav2 configured)
- One-command tmux startup automation implemented
- RViz real-time visualization working
- Hardware NOT assembled yet (bench testing only)

**System:**
- Pi 4 (192.168.86.20) running Ubuntu 22.04 + ROS 2 Humble
- Dev machine: Ubuntu 22.04 + ROS 2 Humble
- GitHub: https://github.com/Dasovon/hoverbot (pi4 branch)

See docs/pi4/PROJECT_STATUS.md for complete details.

**Current Goal:** [State what you want to work on]
```

## Last Updated
December 27, 2025 - Complete software stack validated and documented
