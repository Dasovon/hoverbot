# HoverBot Project Roadmap

## ✅ Completed (Weeks 1-3)

### Week 1-2: Software Development
- [x] ROS 2 Humble setup on Raspberry Pi 4
- [x] Hoverboard driver integration (UART protocol)
- [x] Sensor integration (LiDAR, IMU, cameras)
- [x] Sensor fusion (robot_localization EKF)
- [x] SLAM Toolbox configuration
- [x] Complete launch file system
- [x] 7-sensor operational system

### Week 3: Hardware & Power Optimization
- [x] USB 3.0 cable upgrade (15 Hz → 42 Hz depth performance)
- [x] Dual-camera system validation (RealSense + ELP)
- [x] LiDAR power management system
  - Auto-stop on idle (2.3W savings)
  - Activity-based control
  - Bench test mode
  - Watchdog timeout (30s)
- [x] Complete system integration testing
- [x] SLAM verification in RViz2

**System Status:** Production-ready software stack ✓

---

## 📋 Next Steps (Weeks 4-8)

### 1. Gazebo Simulation Deep Dive
**Priority:** High  
**Timeline:** 1-2 weeks  
**Status:** Not started

**Goals:**
- Create accurate URDF model of HoverBot
  - Dual camera system (RealSense + ELP)
  - RPLidar A1 sensor
  - BNO055 IMU
  - Hoverboard differential drive
  - Correct mass, inertia, collision properties
- Set up Gazebo world for testing
  - Indoor environment (hallways, rooms)
  - Obstacles for navigation testing
  - Realistic physics parameters
- Validate sensor simulation
  - Depth camera simulation
  - 2D LiDAR simulation
  - IMU simulation
  - Odometry from differential drive
- Test navigation stack virtually
  - SLAM mapping in simulation
  - Nav2 navigation stack
  - Parameter tuning without hardware risk
  - Path planning algorithms

**Benefits:**
- Test algorithms safely before physical assembly
- Tune parameters without wearing out hardware
- Develop emergency behaviors in controlled environment
- Validate mechanical design before building

**Resources:**
- Gazebo Classic or Gazebo Fortress
- URDF/Xacro documentation
- Nav2 simulation tutorials
- ROS 2 Control simulation

---

### 2. ros2_control Research & Implementation
**Priority:** Medium  
**Timeline:** 1 week  
**Status:** Not started

**Goals:**
- Research ros2_control framework
  - Hardware abstraction layer
  - Controller manager
  - Standard controller interfaces
- Evaluate if ros2_control fits HoverBot architecture
  - Custom hoverboard UART protocol
  - Differential drive controller
  - Integration with existing driver
- Implement if beneficial
  - Create hardware interface for hoverboard
  - Configure diff_drive_controller
  - Test command/feedback loop

**Decision Point:**
- Current custom driver works well
- ros2_control adds standardization
- Evaluate cost/benefit before implementing

**Resources:**
- ros2_control documentation
- Differential drive controller examples
- Custom hardware interface tutorials

---

### 3. Power Distribution System Design
**Priority:** High (required for physical assembly)  
**Timeline:** 1 week  
**Status:** Not started

**Hardware:**
- UCTRONICS Buck Converter (9-36V → 5V 5A)
  - Input: Hoverboard 36V battery
  - Output: Dual USB ports for Pi + sensors
  - Efficiency: ~85-90%
  - Protection: Overcurrent, overvoltage, reverse polarity

**Goals:**
- Design clean power distribution
  - Buck converter mounting location
  - Wire routing from battery
  - Fusing and protection
  - XT60 connector for easy disconnect
- Calculate power budget
  - Raspberry Pi 4: 3A max (15W)
  - RealSense D435: 0.5A (2.5W)
  - ELP camera: 0.5A (2.5W)
  - RPLidar A1: 0.5A idle, 0.65A scanning (2.5W-3.3W)
  - BNO055 IMU: 0.02A (0.1W)
  - **Total: ~4.6A peak (23W)**
  - Buck converter rated 5A - comfortable margin
- Safety features
  - 5A fuse on 5V rail
  - Emergency stop circuit
  - Battery voltage monitoring
  - Low-voltage cutoff to protect LiPo

**Physical Design:**
- Mount buck converter to hoverboard deck
- Route 36V from battery terminal (use existing power button circuit)
- Dual USB output to Pi + USB hub for sensors
- Clean cable management

**Testing Plan:**
- Bench test buck converter with multimeter
- Verify voltage regulation under load
- Test with all sensors powered
- Measure efficiency and heat dissipation

---

### 4. Migration to Claude Code + GitHub CLI
**Priority:** Medium  
**Timeline:** 1-2 days  
**Status:** Not started

**Goals:**
- Set up direct GitHub repository access
  - Install GitHub CLI on Dev machine
  - Configure authentication and permissions
  - Test push/pull from command line
- Configure Claude Code integration
  - Set up Claude Code CLI tool
  - Link to HoverBot repository
  - Test code generation and commits
  - Establish workflow for AI-assisted development

**Benefits:**
- Streamlined development workflow
- Direct code modifications with AI assistance
- Faster iteration on robot code
- Better version control integration
- Automated testing and validation

**Workflow:**
1. Use Claude Code for development tasks
2. AI generates/modifies code directly in repo
3. Review changes with git diff
4. Test on Pi hardware
5. Commit and push via GitHub CLI

**Setup Steps:**
- Install: `sudo apt install gh` (GitHub CLI)
- Authenticate: `gh auth login`
- Clone with CLI: `gh repo clone Dasovon/hoverbot`
- Configure Claude Code access
- Test workflow with small change

---

## 📦 Week 5-6: Physical Assembly
**Status:** Pending (after simulation + power system)

**Tasks:**
- Mount all sensors to hoverboard platform
- Install power distribution system
- Wire all components
- Cable management
- Structural reinforcement if needed
- Weight distribution optimization

---

## 🚀 Week 7-8: Real-World Testing
**Status:** Pending (after assembly)

**Tasks:**
- Initial power-on tests
- Sensor validation in real environment
- SLAM mapping of real spaces
- Navigation stack tuning
- Emergency stop testing
- Autonomous navigation demos

---

## 📊 Success Metrics

**Software (✓ Complete):**
- All 7 sensors publishing reliably
- SLAM building accurate maps
- Navigation stack functional
- Power management operational

**Hardware (In Progress):**
- Stable 5V power to all components
- Clean cable routing
- Robust sensor mounting
- Safe battery management

**Integration (Future):**
- 10+ minute autonomous operation
- Accurate indoor navigation
- Obstacle avoidance
- Return-to-home capability

---

## 🔄 Current Sprint (Next 2 Weeks)

**Focus:** Simulation + Power System

1. **Gazebo URDF Creation** (Week 1)
   - Build robot model
   - Test in empty world
   - Validate sensor plugins

2. **Navigation in Simulation** (Week 1-2)
   - Test SLAM in Gazebo
   - Configure Nav2
   - Tune parameters

3. **Power System Design** (Week 2)
   - Order UCTRONICS buck converter
   - Design mounting system
   - Plan wire routing

4. **Claude Code Migration** (Week 2)
   - Set up GitHub CLI
   - Configure Claude Code
   - Test workflow

---

## 📝 Notes

**Decisions Made:**
- Dual-camera approach (RealSense depth + ELP RGB) confirmed after Pi 4 USB testing
- Bench test mode essential for pre-assembly development
- SLAM working reliably - ready for navigation stack

**Lessons Learned:**
- Sequential startup timing critical for sensor stability
- Power management saves significant battery life
- USB 3.0 dramatically improves depth camera performance
- EKF sensor fusion essential for stable odometry

**Open Questions:**
- Best Gazebo version for ROS 2 Humble? (Classic vs Fortress)
- ros2_control worth the migration effort?
- Additional sensors needed for outdoor navigation?
