# HoverBot Development Context - Session 2026-01-02

**Purpose:** Complete context transfer for ongoing development
**Last Updated:** 2026-01-02
**Status:** Active Development - Multi-Platform Support Complete

---

## 🎯 Quick Reference

### Development Setup
- **Dev Machine:** Ubuntu desktop with VSCode
- **Robot:** Raspberry Pi 4 @ `ssh hoverbot` (Ubuntu 22.04 + ROS 2 Humble)
- **Workflow:** Dev pushes to GitHub → Pi pulls
- **AI Assistance:** Claude (this chat) + Claude Code (for code generation)

### Current Branch Strategy
- **main:** Production-ready, stable code
- **Feature branches:** Short-lived, merged fast (trunk-based development)

### Key Commands
```bash
# Dev machine
cd ~/hoverbot
git checkout -b feature/name
# ... make changes ...
git push origin feature/name
gh pr create --base main --head feature/name

# Pi 4 machine
ssh hoverbot
cd ~/hoverbot
git checkout main
git pull origin main
cd ros2_ws && colcon build && source install/setup.bash
```

---

## 📚 Complete Session Summary

### Session Context (2026-01-02)

**Starting Point:**
- Successfully completed LiDAR power management (2.3W savings)
- Bench test mode operational
- USB 3.0 camera upgrade (15 Hz → 42 Hz depth performance)
- All 7 sensors integrated and working
- SLAM mapping validated in RViz2

**Major Accomplishments This Session:**

1. **GitHub CLI Setup** ✅
   - Installed and authenticated gh CLI on Dev machine
   - Password-free git operations
   - Fast PR creation/merging workflow

2. **Claude Code Integration** ✅
   - Set up Claude Code web interface
   - Connected to Dasovon/hoverbot repository
   - Successfully generated Gazebo simulation in ONE session
   - Created: hoverbot.gazebo.xacro, gazebo.launch.py, README.md

3. **Workflow Documentation** ✅
   - Created comprehensive CLAUDE_CODE_WORKFLOW.md
   - Based on Anthropic's best practices
   - Trunk-based development strategy
   - Small, atomic commits
   - Fast feedback loops

4. **Multi-Platform Support** ✅
   - Restructured repo with platforms/ directory
   - Platform-specific configs for Pi4, Pi5, Jetson Nano
   - Platform-independent ROS 2 code
   - CHANGELOG.md, CONTRIBUTING.md, SECURITY.md added

5. **Repository Analysis** ✅
   - Sandi Metz-style code review completed
   - Grade: A- (Excellent foundation)
   - Identified critical issue: README.md merge conflict
   - Recommended testing strategy

---

## 🚀 Current Project State

### Hardware Status

**Active Platform: Raspberry Pi 4**
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Serial: /dev/ttyAMA0
- Status: ✅ Production software complete, awaiting physical assembly

**Future Platforms:**
- Pi 5: Documented, not yet configured
- Jetson Nano: Experimental support, untested

**Sensors (All Working):**
1. Hoverboard odometry: 50 Hz
2. BNO055 IMU: 20 Hz (fully calibrated, 9.80 m/s² gravity)
3. RPLidar A1: 7.6 Hz (with power management)
4. RealSense D435: 42 Hz depth (USB 3.0 cable upgrade)
5. ELP USB camera: 16 Hz RGB
6. EKF sensor fusion: 49 Hz
7. SLAM Toolbox: Active, mapping validated

### Software Status

**ROS 2 Packages:**
```
ros2_ws/src/
├── hoverbot_driver/         # Serial communication (99.3% success rate)
├── hoverbot_bringup/        # Launch orchestration (7 sensors)
└── hoverbot_description/    # URDF + Gazebo simulation
```

**Key Features:**
- ✅ Single-command launch: `ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py`
- ✅ Bench test mode: `bench_test_mode:=true`
- ✅ LiDAR power management: Auto-stop when idle (2.3W savings)
- ✅ Tmux automation: 5-terminal startup in one command
- ✅ Persistent device names: udev rules for all USB devices
- ✅ Gazebo simulation: Complete URDF with all sensors

**Documentation:**
- ✅ CLAUDE_CODE_WORKFLOW.md (comprehensive development guide)
- ✅ PROJECT_ROADMAP.md (next steps documented)
- ✅ PLATFORM_SETUP_GUIDE.md (multi-platform comparison)
- ✅ SERIAL_PORT_GUIDE.md (platform-specific serial config)
- ✅ RVIZ2_TROUBLESHOOTING.md (yellow overlay fix, etc.)

---

## 🛠️ Development Workflow

### Established Pattern (Trunk-Based)

**1. Planning (claude.ai chat - 5 min)**
- Discuss feature/approach
- Draft explicit instructions for Claude Code
- Identify constraints and files to modify

**2. Create Branch (Terminal - 1 min)**
```bash
git checkout main
git pull origin main
git checkout -b feature/descriptive-name
```

**3. Update Context (2 min)**
- Edit `.scratchpad/CONTEXT_2026-01-02.md`
- Add current focus, constraints
- Commit context update

**4. Execute (Claude Code - 5-10 min)**
- Paste exact instructions from planning phase
- Reference: "Read .scratchpad/CONTEXT_2026-01-02.md first"
- Review changes before accepting
- Let Claude Code commit

**5. Test (Local - 5 min)**
```bash
# Dev machine
cd ~/hoverbot/ros2_ws
colcon build --packages-select [package]
source install/setup.bash
# Test locally

# Pi 4 machine (if hardware needed)
ssh hoverbot
cd ~/hoverbot
git pull origin feature/name
cd ros2_ws && colcon build && source install/setup.bash
ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py
```

**6. Merge (3 min)**
```bash
git push origin feature/name
gh pr create --base main --head feature/name
gh pr merge --squash
git checkout main && git pull
```

**Total: ~25 minutes per feature**

---

## 🔧 Critical Technical Details

### Firmware Configuration (DO NOT CHANGE)

**Hoverboard Controller:**
- BOARD_VARIANT = 1 (CRITICAL - wrong variant causes power failure)
- Platform: STM32F103RCT6
- Serial: USART3, 115200 baud, 8-byte packets
- Protocol: 0xABCD start frame, 50 Hz command rate
- Timeout: 800ms with safety beeping

**Pin Mappings (BOARD_VARIANT=1):**
- Power button: PB9
- Power latch: PC15
- Buzzer: PC13
- DC link: PA1

### ROS 2 Launch Timing (CRITICAL)

**Sequential startup prevents crashes:**
```
[0s]   Hoverboard driver
[2s]   Static transforms (laser)
[2.5s] Static transforms (camera)
[3s]   IMU (BNO055)
[5s]   Sensor fusion (EKF)
[8s]   RealSense D435
[8.5s] ELP USB camera
[15s]  RPLidar A1 (with power mgmt)
[17s]  SLAM Toolbox
```

**Why timing matters:**
- RPLidar crashes with buffer overflow if started too early
- SLAM drops messages without proper transform initialization
- EKF needs sensor data flowing before publishing transforms

### Power Management System

**LiDAR Manager:**
- Auto-stop on idle: Saves 2.3W
- Activity timeout: 30s
- Watchdog: Prevents stuck motor
- Bench test mode: Disables auto-stop for testing

**Usage:**
```bash
# Normal mode (auto power management)
ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py

# Bench test mode (motor stays on)
ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py bench_test_mode:=true
```

### Sensor Fusion Configuration

**robot_localization EKF:**
- Frequency: 50 Hz
- Inputs: Wheel odometry + IMU
- Output: /odometry/filtered
- Publishes: odom → base_link transform

**Transform Tree:**
```
map → odom → base_link → laser
                       → camera_link
                       → imu_link
```

- map → odom: SLAM Toolbox (7.6 Hz)
- odom → base_link: EKF (49 Hz)
- base_link → sensors: Static transforms

---

## 📦 Repository Structure Analysis

### Sandi Metz Code Review (Grade: A-)

**Strengths:**
- ⭐⭐⭐⭐⭐ Separation of concerns (serial_interface, kinematics, ROS node)
- ⭐⭐⭐⭐⭐ Platform abstraction (platforms/ directory)
- ⭐⭐⭐⭐ Dependency management (clean, minimal)
- ⭐⭐⭐⭐ Documentation (comprehensive, well-organized)

**Critical Issues:**
- 🔴 README.md has merge conflict markers (BLOCKER)
- ⚠️ Duplicate scripts between pi4/pi5 (setup_uart.sh)
- ⚠️ Missing unit tests for kinematics
- ⚠️ Robot params mixed with platform params in configs

**Recommended Priorities:**
1. **IMMEDIATE:** Fix README.md merge conflict
2. **HIGH:** Add unit tests for differential_drive_controller.py
3. **MEDIUM:** Consolidate platform scripts
4. **MEDIUM:** Separate robot_params.yaml from platform configs

### Platform Structure
```
platforms/
├── common/              # Shared utilities
│   ├── scripts/
│   │   └── test_serial.py
│   └── config/
├── raspberry-pi4/       # Pi 4 (Ubuntu 22.04 + Humble)
│   ├── config/
│   │   └── hoverbot_driver.yaml  # /dev/ttyAMA0
│   ├── scripts/
│   │   └── setup_uart.sh
│   └── README.md
├── raspberry-pi5/       # Pi 5 (Ubuntu 24.04 + Jazzy)
│   ├── config/
│   │   └── hoverbot_driver.yaml  # /dev/ttyAMA0
│   ├── scripts/
│   │   ├── setup_uart.sh
│   │   └── test_serial.py
│   └── docs/
│       └── SETUP.md
└── jetson-nano/         # Jetson (JetPack + Humble)
    ├── config/
    │   └── hoverbot_driver.yaml  # /dev/ttyTHS1
    ├── scripts/
    │   └── setup_uart.sh
    └── README.md
```

### ROS 2 Package Organization
```
ros2_ws/src/
├── hoverbot_driver/
│   ├── hoverbot_driver/
│   │   ├── __init__.py
│   │   ├── serial_interface.py              # Hardware abstraction
│   │   ├── differential_drive_controller.py # Kinematics (TESTABLE)
│   │   └── hoverbot_driver_node.py          # ROS orchestration
│   ├── config/
│   │   └── hoverbot_driver.yaml             # Base config
│   ├── launch/
│   └── test/                                 # ← Missing!
│
├── hoverbot_bringup/
│   ├── hoverbot_bringup/
│   │   ├── __init__.py
│   │   └── lidar_manager.py                 # Power management
│   ├── config/
│   │   ├── ekf.yaml                         # Sensor fusion
│   │   ├── slam_toolbox_params.yaml
│   │   └── nav2/
│   │       └── nav2_params.yaml
│   └── launch/
│       ├── hoverbot_full_v3.launch.py       # Main launch
│       ├── lidar_power_management.launch.py
│       └── tmux_startup.launch.py
│
└── hoverbot_description/
    ├── urdf/
    │   ├── hoverbot.urdf                    # Base URDF
    │   └── hoverbot.gazebo.xacro            # Gazebo plugins
    ├── launch/
    │   └── gazebo.launch.py                 # Simulation
    └── README.md
```

---

## 🎓 Key Learnings & Patterns

### What Works (Keep Doing)

1. **Sequential Component Startup**
   - Prevents sensor crashes
   - Allows proper initialization
   - Documented in launch files

2. **Power Management**
   - LiDAR auto-stop saves battery
   - Bench test mode for development
   - Activity-based control

3. **Platform Abstraction**
   - ROS 2 code is platform-independent
   - Only config files change per platform
   - Easy to add new platforms

4. **Trunk-Based Development**
   - Small, fast feature branches
   - Merge to main quickly
   - Single source of truth

5. **Context Documentation**
   - Explicit constraints in CLAUDE_CONTEXT.md
   - Instructions reference context
   - Prevents hallucinations

### What to Avoid

1. **Long-Lived Branches**
   - Causes merge conflicts
   - Context drift
   - Integration pain

2. **Vague Instructions to Claude Code**
   - "Improve this" doesn't work
   - "Add X to line Y in file Z" works great
   - Always specify constraints

3. **Changing Firmware Without Understanding**
   - BOARD_VARIANT=1 is critical
   - Power latch behavior is fragile
   - Document before changing

4. **Starting All Sensors Simultaneously**
   - Causes buffer overflows
   - Transform initialization fails
   - Use sequential timing

5. **Assuming Claude Code Remembers**
   - Each session is stateless
   - Always reference context docs
   - Repeat critical constraints

---

## 🔮 Project Roadmap

### Completed (v0.2.0)

- ✅ Multi-platform support (Pi4, Pi5, Jetson)
- ✅ Complete Gazebo simulation
- ✅ LiDAR power management
- ✅ Dual camera system (RealSense + ELP)
- ✅ SLAM mapping validated
- ✅ Bench test mode
- ✅ USB 3.0 optimization
- ✅ Workflow documentation
- ✅ GitHub community files

### In Progress

- 🔄 README.md merge conflict fix (CRITICAL)
- 🔄 Repository analysis complete, fixes pending

### Next (v0.3.0 - Planned)

**Immediate (This Week):**
1. Fix README.md merge conflict
2. Add unit tests for kinematics
3. Consolidate platform scripts
4. Test Gazebo simulation on Dev machine

**Short-term (This Month):**
1. Nav2 parameter tuning in simulation
2. Autonomous navigation testing (Gazebo)
3. Power distribution system design (UCTRONICS buck converter)
4. Physical assembly planning

**Medium-term (Next 2-3 Months):**
1. ros2_control research and evaluation
2. Physical robot assembly
3. Real-world SLAM testing
4. Performance benchmarking (Pi4 vs Pi5)

### Long-term Vision

- Multi-robot coordination
- Computer vision (Jetson-specific)
- Web interface for monitoring
- Autonomous mapping demos
- Open-source community contributions

---

## 🧰 Tool Setup & Configuration

### Development Tools

**VSCode (Dev Machine):**
- Extensions: ROS, Python, Git
- Terminal integrated
- SSH to Pi 4 for testing

**Claude Code:**
- Web interface at code.claude.com
- Connected to Dasovon/hoverbot repo
- Use for: File creation, code generation
- Pattern: Explicit instructions, reference context

**GitHub CLI:**
- Installed and authenticated
- Password-free operations
- Commands: `gh pr create`, `gh pr merge`, `gh repo view`

**Git Workflow:**
```bash
# Dev machine
git checkout -b feature/name
# ... work ...
git push origin feature/name
gh pr create

# Pi 4 machine
ssh hoverbot
git pull origin main
```

### Claude Code Best Practices

**Always Include:**
1. "Read .scratchpad/CONTEXT_2026-01-02.md first"
2. Explicit constraints: "DO NOT modify X"
3. Expected outcome
4. Success criteria

**Good Instruction Example:**
```
Create nav2_params.yaml in ros2_ws/src/hoverbot_bringup/config/nav2/

Specifications:
- Controller frequency: 20 Hz
- Planner: DWB controller
- Recovery behaviors: standard set

Constraints:
- DO NOT modify existing launch files
- PRESERVE SLAM configuration
- MAINTAIN ROS 2 Humble compatibility

Expected output:
- config/nav2/nav2_params.yaml (new file)

Reference .scratchpad/CONTEXT_2026-01-02.md for project constraints.
```

**Bad Instruction Example:**
```
"Make the navigation better"
"Fix the code"
"Improve performance"
```

---

## 📋 Common Tasks Reference

### Launch System on Pi 4
```bash
ssh hoverbot
cd ~/hoverbot/ros2_ws
source install/setup.bash

# Full system
ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py

# With bench test mode
ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py bench_test_mode:=true

# Start motor manually (bench test mode)
ros2 topic pub --once /robot_active std_msgs/msg/Bool "data: true"
```

### Test in Gazebo (Dev Machine)
```bash
cd ~/hoverbot/ros2_ws
source install/setup.bash

# Launch simulation
ros2 launch hoverbot_description gazebo.launch.py

# Control robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Visualize
rviz2
```

### Check Sensor Status
```bash
# Odometry
ros2 topic hz /odom                    # Should be ~50 Hz

# IMU
ros2 topic hz /bno055/imu              # Should be ~20 Hz

# LiDAR
ros2 topic hz /scan                    # Should be ~7.6 Hz

# Cameras
ros2 topic hz /camera/camera/depth/image_rect_raw  # ~42 Hz
ros2 topic hz /elp/image_raw           # ~16 Hz

# Sensor fusion
ros2 topic hz /odometry/filtered       # Should be ~35 Hz

# SLAM
ros2 topic hz /map                     # Slow (~1 Hz or less)
```

### Build Specific Package
```bash
cd ~/hoverbot/ros2_ws

# Single package
colcon build --packages-select hoverbot_driver

# Clean rebuild
rm -rf build install log
colcon build

# With verbose output
colcon build --event-handlers console_direct+
```

### Git Operations
```bash
# Create feature branch
git checkout main
git pull origin main
git checkout -b feature/descriptive-name

# Commit changes
git add .
git commit -m "feat: Add feature description"

# Push and create PR
git push origin feature/descriptive-name
gh pr create --base main --head feature/descriptive-name \
  --title "feat: Add feature" \
  --body "Description of changes"

# Merge PR
gh pr merge --squash

# Cleanup
git checkout main
git pull origin main
git branch -d feature/descriptive-name
```

---

## 🚨 Known Issues & Workarounds

### README.md Merge Conflict (CRITICAL)

**Issue:** Merge conflict markers in README.md
```markdown
<<<<<<< HEAD
...old content...
=======
...new content...
>>>>>>> claude/change-branch-ZGp4e
```

**Impact:** Can't be rendered on GitHub, confusing for users

**Fix:** Manually resolve conflict, choose better structure (appears to be claude branch version)

**Priority:** IMMEDIATE

---

### RViz2 Yellow Overlay

**Issue:** Yellow/opaque overlay obscuring visualization

**Cause:** Odometry covariance visualization enabled

**Fix:**
1. In RViz2, expand "Odometry" display
2. Expand "Covariance" section
3. Uncheck "Orientation" checkbox

**Documented in:** `docs/RVIZ2_TROUBLESHOOTING.md`

---

### LiDAR Not Starting on First Launch

**Issue:** RPLidar sometimes doesn't start on first system launch

**Cause:** USB enumeration timing

**Workaround:**
```bash
# Restart LiDAR power management
ros2 topic pub --once /robot_active std_msgs/msg/Bool "data: true"
```

**Future Fix:** Add retry logic to lidar_manager.py

---

### EKF Slow to Publish First Transform

**Issue:** Takes ~2 seconds for first odom→base_link transform

**Cause:** Waiting for sensor data from both odometry and IMU

**Not a bug:** Expected behavior, sensors need to publish first

**Workaround:** Launch timing already accounts for this (EKF at 5s)

---

## 💡 Tips for New AI Chat Session

### First Message Template
```
Hi! I'm continuing development on the HoverBot autonomous robot project.

Context file: [paste .scratchpad/CONTEXT_2026-01-02.md]
Repository: Dasovon/hoverbot (main branch)

Current focus: [your current task]

I'm working on Dev machine (Ubuntu desktop) with VSCode, and the robot is a Raspberry Pi 4 at ssh hoverbot. I push to GitHub from Dev, and Pi pulls updates.

I'll be using Claude Code for code generation and you (claude.ai chat) for planning, architecture decisions, and troubleshooting.

Can you confirm you understand the project structure and current state?
```

### How to Use This Context

**For Planning:**
- Reference: "Check .scratchpad/CONTEXT_2026-01-02.md - what are the constraints for [feature]?"
- Strategy: "Based on the project roadmap, what should I tackle next?"

**For Claude Code Instructions:**
- Always start with: "Read .scratchpad/CONTEXT_2026-01-02.md first"
- Include: Explicit constraints from context file
- Reference: Platform-specific details from context

**For Troubleshooting:**
- Check: "Known Issues & Workarounds" section first
- Reference: "Critical Technical Details" for firmware/timing issues
- Search: This context file for similar problems

### Update This File

**When to update:**
- After completing a feature
- When discovering new constraints
- After major architectural changes
- When adding new platforms

**How to update:**
```bash
cd ~/hoverbot/.scratchpad
nano CONTEXT_2026-01-02.md
# Make updates
git add .scratchpad/CONTEXT_2026-01-02.md
git commit -m "docs: Update context with [what changed]"
git push origin main
```

---

## 📖 Essential Reading (In Order)

**Before starting new features:**
1. This context file (you're reading it!)
2. `docs/CLAUDE_CODE_WORKFLOW.md` - Development workflow
3. `docs/PROJECT_ROADMAP.md` - Next steps
4. Platform-specific README in `platforms/[your-platform]/`

**For specific tasks:**
- Adding features: `CONTRIBUTING.md`
- Debugging: `docs/RVIZ2_TROUBLESHOOTING.md`, Known Issues section above
- Hardware: `hardware/README.md`, `docs/FIRMWARE.md`
- SLAM tuning: `docs/pi4/PARAMETER_TUNING.md`

**For architecture decisions:**
- Sandi Metz analysis (this file, "Repository Structure Analysis" section)
- Dependency diagrams in ROS 2 package sections
- Transform tree in "Sensor Fusion Configuration"

---

## 🎯 Success Criteria

**You're doing it right when:**

1. **Features take < 1 hour** from idea to merged
2. **PRs are < 200 lines** of changes
3. **Branches live < 3 days**
4. **Main branch always builds** without errors
5. **Tests pass** before merging (when tests exist)
6. **Documentation updated** with code changes
7. **Context file current** (updated this week)
8. **No merge conflicts** (frequent small merges prevent this)

**Warning signs:**

- Branch open > 1 week
- PR with 1000+ line changes
- Merge conflicts on every PR
- Tests breaking after merges
- "Works on my machine" (test on both Dev and Pi4!)

---

## 🔐 Security & Credentials

**Never commit:**
- SSH private keys
- API keys
- Passwords
- Personal tokens

**Safe to commit:**
- Public keys (in docs)
- Configuration files (no secrets)
- Launch files
- URDF/robot models

**GitHub authentication:**
- Using GitHub CLI with OAuth
- No passwords in git config
- Credentials managed by gh CLI

---

## 📞 Getting Help

**In this chat:**
- Planning: "I want to add [feature], how should I approach it?"
- Debugging: "Getting error [X], context says [Y], what's wrong?"
- Architecture: "Should I use pattern [A] or [B] for [task]?"

**With Claude Code:**
- Code generation: Paste explicit instructions with context reference
- File creation: Specify exact path, format, contents
- Refactoring: List constraints, files to preserve

**GitHub Issues:**
- Bugs: Use bug report template
- Features: Use feature request template
- Questions: Use discussions

**When stuck:**
1. Check this context file
2. Check Known Issues section
3. Check platform-specific README
4. Ask in chat with full error context
5. Review similar code in repository
6. Create GitHub issue if novel problem

---

## 📅 Version History

**This Context File:**
- **v1.0** - 2026-01-02: Initial creation from complete session
- Contains: Full chat history, Sandi Metz analysis, workflow, roadmap
- Status: Ready for new AI chat session

**Project Versions:**
- **v0.1.0**: Initial implementation (Dec 2024)
- **v0.2.0**: Multi-platform support, Gazebo sim (Jan 2026) ← Current
- **v0.3.0**: Planned (Nav2, testing, physical assembly)

---

## 🎓 Final Reminders

**Core Principles:**
1. Small, atomic changes
2. Explicit instructions to AI
3. Reference context always
4. Test before merging
5. Main branch always stable

**Workflow Mantra:**
> "Plan in chat, execute in Claude Code, test locally, merge fast"

**Sandi Metz Wisdom:**
> "Make the change easy, then make the easy change"

**Project Goal:**
Build a production-quality autonomous robot with:
- Reliable navigation
- Multiple platform support
- Clean, maintainable code
- Comprehensive documentation
- Active open-source community

---

**You now have complete context to continue development. Good luck! 🤖🚀**
