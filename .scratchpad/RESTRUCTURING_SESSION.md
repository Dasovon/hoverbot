# HoverBot Repository Restructuring - Session Scratchpad

**Date Started:** 2026-01-01
**Objective:** Restructure repository for multi-platform support (Pi4, Pi5, Jetson Nano)

---

## Session Progress

### Current Status: STARTED

**Branch:** `claude/change-branch-ZGp4e`

---

## Restructuring Plan

### Phase 1: Create Platform Structure ✅ Starting
- [ ] Create `platforms/` root directory
- [ ] Create `platforms/common/` for shared utilities
- [ ] Create `platforms/raspberry-pi5/` (current Pi5 setup)
- [ ] Create `platforms/raspberry-pi4/` (new Pi4 support)
- [ ] Create `platforms/jetson-nano/` (new Jetson support)

### Phase 2: Migrate Existing Files
- [ ] Move `raspberry-pi/` → `platforms/raspberry-pi5/`
- [ ] Move `docs/SETUP.md` → `platforms/raspberry-pi5/docs/SETUP.md`
- [ ] Move relevant Pi5-specific docs

### Phase 3: Create Platform-Specific Configs
- [ ] Create Pi5 config files
- [ ] Create Pi4 config files (adapted from Pi5)
- [ ] Create Jetson config files (new serial port)

### Phase 4: Create Documentation
- [ ] Pi5 comprehensive setup guide (Ubuntu 24.04 + Jazzy)
- [ ] Pi4 comprehensive setup guide (Ubuntu 22.04 + Humble)
- [ ] Jetson Nano setup guide (JetPack + Humble)
- [ ] Common platform guide
- [ ] Serial port configuration guide
- [ ] Update main README

### Phase 5: Validation & Testing
- [ ] Verify no broken links
- [ ] Test that Pi5 setup still works
- [ ] Commit and push changes

---

## Platform Serial Port Mapping

| Platform | Serial Device | ROS Distribution | Ubuntu Version |
|----------|---------------|------------------|----------------|
| Raspberry Pi 4 | `/dev/ttyAMA0` | Humble | 22.04 LTS |
| Raspberry Pi 5 | `/dev/ttyAMA0` | Jazzy | 24.04 LTS |
| Jetson Nano | `/dev/ttyTHS1` | Humble | 20.04 (JetPack) |

---

## Files Created This Session

### Directories
- [ ] `platforms/`
- [ ] `platforms/common/`
- [ ] `platforms/raspberry-pi4/`
- [ ] `platforms/raspberry-pi5/`
- [ ] `platforms/jetson-nano/`

### Documentation Files
- [ ] `platforms/raspberry-pi5/docs/SETUP.md`
- [ ] `platforms/raspberry-pi4/docs/SETUP.md`
- [ ] `platforms/jetson-nano/docs/SETUP.md`
- [ ] `platforms/common/README.md`
- [ ] `docs/PLATFORM_SETUP_GUIDE.md`
- [ ] `docs/SERIAL_PORT_GUIDE.md`

### Config Files
- [ ] `platforms/raspberry-pi5/config/hoverbot_driver.yaml`
- [ ] `platforms/raspberry-pi4/config/hoverbot_driver.yaml`
- [ ] `platforms/jetson-nano/config/hoverbot_driver.yaml`

---

## Notes & Decisions

### Decision Log
1. **Serial Port Defaults:**
   - Pi4/Pi5: `/dev/ttyAMA0` (same for both)
   - Jetson Nano: `/dev/ttyTHS1` (standard UART)

2. **ROS Distribution Choices:**
   - Pi5: ROS 2 Jazzy (Ubuntu 24.04 native support)
   - Pi4: ROS 2 Humble (Ubuntu 22.04 LTS - stable)
   - Jetson: ROS 2 Humble (best JetPack compatibility)

3. **Directory Structure:**
   - Keep all ROS 2 packages in `ros2_ws/` (platform-independent)
   - Platform-specific files ONLY in `platforms/{platform}/`

### Key Insights
- Current code is already 95% platform-independent
- Only changes needed: serial port paths and setup scripts
- No ROS 2 code modifications required

---

## Commands Reference

### Build and Test
```bash
cd ~/hoverbot/ros2_ws
colcon build
source install/setup.bash
```

### Launch with Platform-Specific Config
```bash
# Pi5
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=/home/user/hoverbot/platforms/raspberry-pi5/config/hoverbot_driver.yaml

# Pi4
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=/home/user/hoverbot/platforms/raspberry-pi4/config/hoverbot_driver.yaml

# Jetson
ros2 launch hoverbot_driver hoverbot_driver.launch.py \
  params_file:=/home/user/hoverbot/platforms/jetson-nano/config/hoverbot_driver.yaml
```

---

## Next Steps

1. Create directory structure
2. Create platform-specific config files
3. Write comprehensive documentation for each platform
4. Update main README with platform selector
5. Test and commit

---

## Session End

**Status:** In Progress
**Next Session:** Continue with platform-specific documentation creation

---

*This scratchpad will be updated throughout the restructuring process.*
