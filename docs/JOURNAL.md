# HoverBot Development Journal

Chronological log of development progress, issues, and solutions.

---

## December 2024 - Project Foundation

### Hardware Assembly
- ✅ Raspberry Pi 4 installed on hoverboard platform
- ✅ RPLidar A1 mounted and wired
- ✅ 36V 10S battery pack integrated
- ✅ UART wiring completed (Pi → right sideboard)

### Firmware Configuration

**Initial Challenge: Power Button Not Latching**
- **Problem:** After flashing VARIANT_USART firmware, power button would not latch
- **Symptoms:** Board powered off immediately when releasing power button
- **Initial hypothesis:** GPIO latch circuit needed manual control
- **Root cause discovered:** Incorrect BOARD_VARIANT setting
- **Solution:** Changed `BOARD_VARIANT` from 0 to 1 in config.h
- **Result:** Power button works normally, board latches properly
- **Key learning:** BOARD_VARIANT controls pin mapping for different mainboard types

**Serial Protocol Reverse Engineering**
- **Challenge:** Documentation unclear on exact packet format
- **Process:**
  1. Examined util.h for SerialCommand structure (lines 28-35)
  2. Found checksum algorithm in util.c (line 1276)
  3. Validated packet size: 8 bytes (not 18 as initially assumed)
  4. Confirmed checksum: `start XOR steer XOR speed`
- **Result:** Working bidirectional communication established

**TANK_STEERING Configuration**
- **Challenge:** Initial commands had no effect on motors
- **Discovery:** TANK_STEERING was commented out in config.h
- **Impact without TANK_STEERING:**
  - Inputs interpreted as speed + steer (mixed output)
  - Required for differential drive (direct wheel control)
- **Solution:** Enabled TANK_STEERING in config.h
- **Result:** Direct left/right wheel control working

**USART Selection**
- **Decision:** Use USART3 (right sideboard) instead of USART2 (left)
- **Rationale:** USART3 is 5V tolerant, safer for Pi's 3.3V GPIO
- **Configuration:** 115200 baud, 8N1, ~50Hz command rate

### Software Development

**ROS 2 Environment**
- **Platform:** Ubuntu 24.04 LTS (22.04 incompatible with Pi 5)
- **ROS Distribution:** Jazzy Jalisco (latest for Ubuntu 24.04)
- **Packages created:**
  - `hoverbot_description` - URDF robot model
  - `hoverbot_bringup` - Launch file architecture

**Python Serial Control**
- **Library:** pyserial
- **Implementation:** Complete command/feedback handling
- **Features:**
  - 8-byte packet construction
  - XOR checksum calculation
  - 50Hz command loop
  - Telemetry feedback parsing
  - Emergency stop on exit

### Validation Testing

**Serial Communication Test**
- ✅ Beeping stops when valid commands received
- ✅ Continuous command stream maintains connection
- ✅ Timeout triggers safety beeping (intentional)
- ✅ Feedback telemetry received at ~50Hz

**Motor Control Test**
- ✅ Forward motion (both wheels same speed)
- ✅ Backward motion (both wheels reverse)
- ✅ Turning (differential wheel speeds)
- ✅ Smooth acceleration and deceleration

**Telemetry Validation**
- ✅ Battery voltage: ~42V (healthy 10S pack)
- ✅ Board temperature: ~26°C (normal operating temp)
- ✅ Wheel speed feedback available
- ✅ Command echo working

---

## Technical Decisions

### Why Ubuntu 24.04?
- Ubuntu 22.04 kernel lacks Pi 5 hardware support
- ROS 2 Jazzy is the official distribution for 24.04
- Better long-term support for Raspberry Pi 4

### Why USART3?
- 5V tolerant GPIO (safer than USART2)
- Right sideboard more accessible for wiring
- Identical protocol to USART2

### Why 50Hz Command Rate?
- Balances responsiveness with CPU overhead
- Well above timeout threshold (1.25Hz minimum)
- Standard for motor control applications
- Matches typical sensor update rates

### Why TANK_STEERING?
- Direct wheel speed control (simpler for ROS)
- No need for steer/speed mixing in software
- Easier odometry calculations
- Standard for differential drive robots

---

## Lessons Learned

### Firmware
1. **Always check BOARD_VARIANT** - Critical for power management
2. **Read actual source code** - Documentation may be incomplete
3. **Timeout beeping is safety feature** - Not a bug to fix
4. **Continuous heartbeat required** - Failsafe is intentional

### Hardware
1. **Hold power button during flashing** - Essential for ST-Link
2. **Use 100 kHz SWD speed** - More reliable than 4 MHz
3. **Test with wheels off ground** - Safety first
4. **Verify voltage levels** - USART3 is 5V tolerant

### Protocol
1. **Packet is 8 bytes, not 18** - Structure definition is authoritative
2. **Little-endian byte order** - Use struct.pack('<HhhH', ...)
3. **Simple XOR checksum** - Not complex CRC
4. **steer/speed naming** - Even in TANK_STEERING mode

### Development Process
1. **Hardware first** - Validate electrical before software
2. **Incremental testing** - One component at a time
3. **Document everything** - Future self will thank you
4. **Version control often** - Small, logical commits

---

## Known Issues

### None Currently
All major issues resolved. System operational and stable.

---

## Next Steps

### Immediate (ROS 2 Driver)
- [ ] Create `hoverbot_driver` package
- [ ] Implement `/cmd_vel` subscriber
- [ ] Add odometry publisher
- [ ] Test with `teleop_twist_keyboard`

### Short Term (Sensors)
- [ ] Integrate RPLidar A1 driver
- [ ] Validate laser scan data
- [ ] Configure transform tree
- [ ] Test in RViz

### Medium Term (SLAM)
- [ ] Configure slam_toolbox
- [ ] Create indoor maps
- [ ] Test localization
- [ ] Save map files

### Long Term (Navigation)
- [ ] Configure Nav2 stack
- [ ] Tune costmap parameters
- [ ] Implement goal navigation
- [ ] Add obstacle avoidance

---

## Resources Referenced

### Official Documentation
- [EFeru FOC Firmware](https://github.com/EFeru/hoverboard-firmware-hack-FOC)
- [ROS 2 Jazzy Docs](https://docs.ros.org/en/jazzy/)
- [RPLidar SDK](https://github.com/Slamtec/rplidar_sdk)

### YouTube Tutorials
- Hoverboard power button fix videos (critical for BOARD_VARIANT discovery)
- ROS 2 differential drive robot tutorials
- SLAM and navigation demonstrations

### Community Forums
- ROS Answers
- Robotics Stack Exchange
- EFeru GitHub Issues

---

**Status:** Foundation complete, ready for ROS 2 integration  
**Last Updated:** December 21, 2024
