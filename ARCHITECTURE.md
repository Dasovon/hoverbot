# HoverBot Architecture Documentation

**Last Updated:** 2026-01-05
**Version:** 1.0
**Target:** Developers and maintainers

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Design Principles](#design-principles)
3. [Module Architecture](#module-architecture)
4. [Critical Design Decisions](#critical-design-decisions)
5. [Timing and Performance](#timing-and-performance)
6. [Hardware Integration](#hardware-integration)
7. [Future Extensibility](#future-extensibility)

---

## System Overview

HoverBot is a ROS2-based autonomous robot built on a hoverboard platform. The driver architecture bridges the gap between ROS2's standard navigation interfaces and the EFeru hoverboard firmware's custom serial protocol.

### Key Components

```
┌─────────────────────────────────────────────────────┐
│                  ROS2 Navigation Stack               │
│              (Nav2, SLAM, Planners, etc.)            │
└─────────────────────────────────────────────────────┘
                        ↕ /cmd_vel (Twist)
┌─────────────────────────────────────────────────────┐
│              HoverBotDriverNode (ROS2)               │
│  ┌───────────────────────────────────────────────┐  │
│  │  DifferentialDriveController (Kinematics)     │  │
│  │  • Twist → wheel velocities                   │  │
│  │  • Wheel velocities → Twist                   │  │
│  └───────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────┐  │
│  │  HoverboardSerialInterface (Protocol)         │  │
│  │  • Command encoding (8 bytes)                 │  │
│  │  • Telemetry decoding (18 bytes)              │  │
│  │  • Frame synchronization                      │  │
│  └───────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
                        ↕ UART (115200 baud)
┌─────────────────────────────────────────────────────┐
│         EFeru Hoverboard Firmware (STM32)            │
│  • FOC motor control                                 │
│  • Tank steering mode                                │
│  • Encoder feedback                                  │
│  • Battery/temperature monitoring                    │
└─────────────────────────────────────────────────────┘
                        ↕ PWM signals
┌─────────────────────────────────────────────────────┐
│                Physical Motors & Sensors             │
└─────────────────────────────────────────────────────┘
```

---

## Design Principles

The architecture follows these core principles:

### 1. **Separation of Concerns**

Each module has a single, well-defined responsibility:

- **`hoverbot_driver_node.py`**: ROS2 orchestration and lifecycle management
- **`differential_drive_controller.py`**: Pure kinematics calculations
- **`serial_interface.py`**: Serial protocol encoding/decoding

This separation enables:
- Independent testing of each component
- Swapping implementations (e.g., different kinematics models)
- Reusing modules in other projects

### 2. **Hardware Abstraction**

The `HoverboardSerialInterface` class abstracts all hardware-specific details:
- Serial port configuration
- Protocol framing
- Byte ordering (little-endian)
- Checksum validation

This means the rest of the code can work with clean Python dataclasses (`HoverboardFeedback`) rather than raw bytes.

### 3. **ROS2 Best Practices**

- **Standard Interfaces**: Uses standard ROS2 message types (`Twist`, `Odometry`, `DiagnosticArray`)
- **TF2 Integration**: Proper transform tree (`odom → base_link`)
- **Parameter Server**: All configuration is parameterizable
- **Quality of Service**: Appropriate QoS settings for real-time control

### 4. **Safety First**

Multiple layers of safety:
- **Command timeout**: Robot stops if no commands received for 0.5s
- **Velocity limiting**: Commands clamped to safe RPM limits
- **Checksum validation**: Corrupted packets are discarded
- **Clean shutdown**: Zero command sent before disconnect

### 5. **Fail-Fast Philosophy**

Rather than trying to recover from unexpected states, the driver:
- Validates data at boundaries (checksums, timeouts)
- Logs errors clearly
- Allows ROS2 lifecycle to handle restarts if needed

---

## Module Architecture

### HoverBotDriverNode (hoverbot_driver_node.py)

**Purpose:** Main ROS2 node that orchestrates all components.

**Responsibilities:**
1. **ROS2 Lifecycle**: Initialization, shutdown, parameter management
2. **Command Processing**: Subscribe to `/cmd_vel`, apply timeout logic
3. **Control Loop**: 50Hz timer driving the main update cycle
4. **Odometry**: Calculate robot pose from wheel speeds
5. **Publishing**: Odometry, TF transforms, diagnostics
6. **Activity Signaling**: Signal robot activity state for peripherals

**Key Design Decisions:**

- **50Hz Control Loop**: Required by firmware (prevents timeout beeping)
  ```python
  self.timer = self.create_timer(0.02, self.control_loop)  # 20ms = 50Hz
  ```

- **Right Wheel Sign Correction**: Hardware quirk requires negation
  ```python
  self.update_odometry(feedback.speed_l_rpm, -feedback.speed_r_rpm)
  ```
  This MUST remain in the driver node, not in the serial interface (it's a vehicle-specific quirk, not a protocol detail).

- **Diagnostics Decimation**: Published at 5Hz (every 10th cycle) to reduce overhead
  ```python
  if self.serial.rx_count % 10 == 0:
      self.publish_diagnostics(feedback)
  ```

**State Variables:**
- Odometry state: `x`, `y`, `theta` (robot pose in odom frame)
- Command state: `current_linear`, `current_angular`, `last_cmd_vel_time`
- Flags: `cmd_vel_received`, `publish_odom_flag`, `publish_tf_flag`

**Future Refactoring:** See `REFACTORING_PROPOSALS.md` for plan to extract `OdometryPublisher` and `DiagnosticsPublisher` classes.

---

### DifferentialDriveController (differential_drive_controller.py)

**Purpose:** Pure kinematics calculations for differential drive robot.

**Responsibilities:**
1. **Inverse Kinematics**: Convert Twist (linear, angular) → wheel speeds (RPM)
2. **Forward Kinematics**: Convert wheel speeds (RPM) → Twist (for odometry)
3. **Limit Checking**: Validate commands against physical constraints
4. **Unit Conversions**: RPM ↔ m/s, firmware commands ↔ RPM

**Key Equations:**

**Inverse Kinematics** (Twist → Wheels):
```
v_left  = v_linear - (ω_angular × wheelbase / 2)
v_right = v_linear + (ω_angular × wheelbase / 2)
```

**Forward Kinematics** (Wheels → Twist):
```
v_linear  = (v_left + v_right) / 2
ω_angular = (v_right - v_left) / wheelbase
```

**Unit Conversions:**
```
RPM → m/s:  v = (RPM × 2π × radius) / 60
m/s → RPM:  RPM = (v × 60) / (2π × radius)
RPM → cmd:  cmd = (RPM / max_RPM) × 1000
```

**Design Rationale:**

This class is **stateless** - all methods are pure functions of their inputs. This makes it:
- Easy to test (no setup required)
- Thread-safe (no shared mutable state)
- Reusable (could be used in simulation, other robots, etc.)

The class stores only **configuration** (wheel diameter, wheelbase, limits), not runtime state.

---

### HoverboardSerialInterface (serial_interface.py)

**Purpose:** Low-level serial protocol implementation.

**Responsibilities:**
1. **Connection Management**: Open, close, check serial port
2. **Command Transmission**: Encode and send 8-byte command packets
3. **Feedback Reception**: Receive and decode 18-byte telemetry packets
4. **Frame Synchronization**: Critical! Align to packet boundaries
5. **Validation**: Checksum verification, error counting
6. **Statistics**: Track TX/RX counts, errors

**Protocol Details:**

**Command Packet (8 bytes, little-endian):**
```
Offset  Type     Field       Description
------  -------  ----------  ---------------------------
0-1     uint16   start       Start frame (0xABCD)
2-3     int16    steer       Left wheel cmd (-1000 to +1000)
4-5     int16    speed       Right wheel cmd (-1000 to +1000)
6-7     uint16   checksum    XOR of all fields
```

**Feedback Packet (18 bytes, little-endian):**
```
Offset  Type     Field       Description
------  -------  ----------  ---------------------------
0-1     uint16   start       Start frame (0xABCD)
2-3     int16    cmd1        Processed left command
4-5     int16    cmd2        Processed right command
6-7     int16    speed_r     Right wheel RPM
8-9     int16    speed_l     Left wheel RPM
10-11   int16    bat_v       Battery voltage × 100
12-13   int16    temp        Temperature × 10
14-15   uint16   led         LED control (unused)
16-17   uint16   checksum    XOR of all fields
```

**Critical: Frame Synchronization**

The most complex part of this module is the frame synchronization algorithm. Here's why it's needed:

**Problem:** Serial data arrives as a continuous byte stream with no guaranteed alignment to packet boundaries. If we naively read 18-byte chunks, we might start mid-packet and never recover sync.

**Solution:** Byte-by-byte scanning for the start frame marker (`0xCD 0xAB`):

```python
# Scan sync buffer for start frame (0xABCD in little-endian = 0xCD 0xAB)
for i in range(len(self.sync_buffer) - self.FEEDBACK_PACKET_SIZE + 1):
    if self.sync_buffer[i] == 0xCD and self.sync_buffer[i+1] == 0xAB:
        # Found potential start frame
        # Try to unpack and validate packet
        # If valid, remove from buffer and return
        # If invalid, keep scanning
```

**Why This Works:**
1. Start frame is unique (0xCD 0xAB is unlikely to occur in data)
2. Checksum validation confirms packet integrity
3. Buffer is trimmed after successful parse to maintain sync
4. Tail bytes are kept in case start frame spans chunk boundary

**Performance:** This approach is efficient because:
- Only scans when data is available
- Stops scanning after first valid packet found
- Buffer is bounded (max 200 bytes) to prevent memory bloat

**⚠️ WARNING:** Do NOT modify this algorithm without extensive hardware testing. It was hard-won through debugging.

---

## Critical Design Decisions

### Decision 1: Why 50Hz Control Loop?

**Context:** The EFeru firmware has a built-in timeout (default 160ms). If it doesn't receive valid commands, it stops the motors and beeps.

**Decision:** Run main control loop at 50Hz (20ms period).

**Rationale:**
- 20ms × 8 cycles = 160ms (firm timeout threshold)
- Provides 8:1 safety margin
- Aligns with common ROS2 control rates
- Fast enough for responsive control
- Slow enough to not overwhelm CPU on Raspberry Pi

**Tradeoffs:**
- ✅ Prevents timeout beeping
- ✅ Responsive control
- ⚠️ Requires reliable timing (ROS2 timer, not Python sleep)
- ⚠️ CPU load on resource-constrained platforms

**Alternative Considered:** 10Hz was too slow (would trigger timeouts if one packet drops), 100Hz was unnecessary overhead.

---

### Decision 2: Where to Negate Right Wheel RPM?

**Context:** The hoverboard firmware reports the right wheel RPM as negative (even when spinning forward). This is likely a firmware bug or wiring convention.

**Decision:** Negate right wheel RPM in the **driver node**, not the serial interface.

**Rationale:**
- Serial interface should report **raw** firmware values
- Driver node handles **vehicle-specific** quirks
- Diagnostics can show raw values for debugging
- If firmware is fixed, only one line changes

**Location:** `hoverbot_driver_node.py:196`
```python
self.update_odometry(feedback.speed_l_rpm, -feedback.speed_r_rpm)
```

**Alternative Considered:** Negating in `serial_interface.py` would hide the quirk but make debugging harder (diagnostics would show corrected values, not raw telemetry).

---

### Decision 3: Odometry Integration Method

**Context:** Need to convert wheel velocities to robot pose over time.

**Decision:** Use Euler integration with midpoint method.

**Implementation:**
```python
delta_theta = angular_z * dt
delta_x = linear_x * math.cos(self.theta + delta_theta / 2.0) * dt
delta_y = linear_x * math.sin(self.theta + delta_theta / 2.0) * dt
```

**Rationale:**
- **Midpoint method** reduces error for curved paths
- Simple to implement and understand
- Computational efficient (no matrix math)
- Accurate enough for dead reckoning (SLAM will correct drift anyway)

**Tradeoffs:**
- ✅ Fast computation
- ✅ Good accuracy for small dt (20ms)
- ⚠️ Accumulates error over time (expected for dead reckoning)
- ⚠️ Not as accurate as Runge-Kutta, but much simpler

**Alternative Considered:** Simple Euler (`theta` not updated mid-step) was less accurate. Runge-Kutta 4 was overkill for this application.

---

### Decision 4: Stateless Kinematics Controller

**Context:** The differential drive controller needs to convert between Twist and wheel speeds.

**Decision:** Make it a **stateless** class with only configuration stored.

**Rationale:**
- **Testability**: Pure functions are easy to test
- **Reusability**: Can be used in different contexts (simulation, multiple robots)
- **Thread safety**: No shared mutable state
- **Predictability**: Same inputs always give same outputs

**Implementation:** All methods are pure functions:
```python
def twist_to_wheels(self, linear_x: float, angular_z: float) -> Tuple[int, int]:
    # No access to instance state except configuration
    # Returns deterministic output for given input
```

**Alternative Considered:** Storing last command in controller was considered but rejected (state belongs in driver node, not kinematics module).

---

## Timing and Performance

### Control Loop Timing

```
Every 20ms (50Hz):
├─ 1. Check cmd_vel timeout (~0.1ms)
├─ 2. Twist → wheel commands (~0.1ms)
├─ 3. Send command to UART (~0.5ms)
├─ 4. Read feedback from UART (~1-2ms)
├─ 5. Update odometry (~0.2ms)
├─ 6. Publish odometry (~0.5ms)
├─ 7. Publish TF (~0.5ms)
└─ 8. Publish diagnostics (every 10th) (~1ms)

Total: ~3-5ms per cycle
Budget: 20ms
Margin: ~15ms (75%)
```

### Telemetry Rate

- **Firmware sends**: 100Hz (every 10ms)
- **Driver reads**: 50Hz (every 20ms)
- **Result**: Driver sees every 2nd telemetry packet (acceptable)

### CPU Usage (Raspberry Pi 4)

Measured on Raspberry Pi 4 (1.5 GHz ARM):
- **Driver node**: ~2-3% CPU (single core)
- **Total ROS2 overhead**: ~5-8% CPU
- **Plenty of headroom** for navigation, SLAM, etc.

### Memory Usage

- **Driver node**: ~30 MB resident set size
- **Sync buffer**: < 1 KB (bounded at 200 bytes)
- **ROS2 middleware**: ~50 MB
- **Total**: ~100 MB (fine for 4-8 GB Pi)

---

## Hardware Integration

### UART Configuration

**Raspberry Pi Side:**
- Device: `/dev/ttyAMA0` (primary UART, GPIO14/15)
- Baud: 115200
- Data: 8 bits
- Parity: None
- Stop: 1 bit
- Flow control: None

**Required `/boot/firmware/config.txt` settings:**
```
enable_uart=1
dtoverlay=disable-bt
```

The Bluetooth overlay must be disabled because it uses the primary UART by default.

### Pin Mapping

```
Raspberry Pi 4       Hoverboard (Right Sideboard)
GPIO14 (TX) ────────→ RX (sensor cable pin 2)
GPIO15 (RX) ←──────── TX (sensor cable pin 3)
GND ─────────────────→ GND (sensor cable pin 1)
```

**Important:** Use the **right** sideboard sensor port (USART3). It is 5V tolerant, unlike USART2.

### Firmware Configuration

The EFeru firmware must be compiled with:
```c
// In Inc/config.h:
#define VARIANT_USART          // Enable USART variant
#define CONTROL_SERIAL_USART3  // Use USART3 (right sensor port)
#define FEEDBACK_SERIAL_USART3 // Send feedback on USART3
#define TANK_STEERING          // Enable tank steering mode
#define BOARD_VARIANT 1        // CRITICAL: Enables power management
```

**BOARD_VARIANT=1 is critical** - without it, the motors may not initialize correctly.

---

## Future Extensibility

### Planned Enhancements

See `REFACTORING_PROPOSALS.md` for detailed plans:

1. **Extract OdometryPublisher**: Separate odometry concerns
2. **Extract DiagnosticsPublisher**: Separate diagnostics concerns
3. **Add IMU Fusion**: Integrate BNO055 data with wheel odometry
4. **Add Slip Detection**: Detect wheel slip and reduce odometry confidence
5. **Add Watchdog**: Restart node if hardware becomes unresponsive

### Extension Points

The architecture is designed to be extended:

**Custom Kinematics:**
```python
# Create subclass with different kinematics model
class MecanumDriveController(DifferentialDriveController):
    def twist_to_wheels(self, linear_x, linear_y, angular_z):
        # Mecanum kinematics (4 wheels)
        pass
```

**Alternative Odometry:**
```python
# Replace dead reckoning with EKF (IMU + wheels)
class EKFOdometry(OdometryPublisher):
    def update_from_wheels_and_imu(self, wheels, imu):
        # Kalman filter fusion
        pass
```

**Custom Serial Protocol:**
```python
# Support different hoverboard firmware
class AlternativeSerialInterface(HoverboardSerialInterface):
    def read_feedback(self):
        # Different packet format
        pass
```

### Plugin Architecture (Future)

Could convert to ROS2 lifecycle nodes with plugin architecture:

```python
# In driver node
self.kinematics = self.create_plugin('kinematics', 'differential_drive')
self.odometry = self.create_plugin('odometry', 'dead_reckoning')
```

This would allow swapping implementations via configuration.

---

## Debugging and Diagnostics

### Key Debugging Tools

1. **Test Script**: `test/test_serial_protocol.py`
   - Standalone test of serial communication
   - Run BEFORE full driver to isolate issues

2. **Diagnostics Topic**: `/diagnostics`
   - Battery voltage
   - Temperature
   - TX/RX counts
   - Error counts

3. **RViz Visualization**:
   - TF tree shows `odom → base_link`
   - Odometry path shows robot trajectory
   - Laser scan (if RPLidar attached)

4. **ROS2 Tools**:
   ```bash
   ros2 topic hz /odom        # Check publishing rate
   ros2 topic echo /odom      # See odometry values
   ros2 node info hoverbot_driver  # Node status
   ```

### Common Issues and Solutions

See `SESSION_SCRATCHPAD.md` section "Known Issues & Workarounds" for hardware-specific debugging.

**Frame Sync Issues:**
- **Symptom**: Checksum errors, framing errors
- **Check**: Baud rate matches (115200)
- **Check**: Wiring (TX→RX, RX→TX)
- **Debug**: Run `test_serial_protocol.py` to verify raw communication

**Odometry Drift:**
- **Symptom**: Robot slowly veers left or right
- **Expected**: Normal for dead reckoning (1-2% error is typical)
- **Solution**: Integrate SLAM for drift correction

**Timeout Beeping:**
- **Symptom**: Hoverboard beeps continuously
- **Check**: Driver node is running and publishing at 50Hz
- **Debug**: `ros2 topic hz /cmd_vel` to check command rate

---

## Summary

The HoverBot architecture prioritizes:

1. **Reliability**: Hardware-validated frame synchronization
2. **Simplicity**: Clear module boundaries, minimal dependencies
3. **Maintainability**: Well-documented design decisions
4. **Extensibility**: Plugin points for future enhancements
5. **Safety**: Multiple layers of validation and timeout handling

The design has been battle-tested with real hardware and represents **hard-won stability**. Any modifications should be made incrementally with thorough testing.

---

**For refactoring proposals, see:** `REFACTORING_PROPOSALS.md`
**For implementation details, see:** Code comments in each module
**For usage instructions, see:** `README.md`
