# HoverBot Refactoring Proposals
## Detailed Before/After Examples

**Date:** 2026-01-05
**Based on:** ANALYSIS_REPORT.md
**Approach:** Incremental, low-risk refactorings following Sandi Metz principles

---

## Table of Contents

1. [Proposal #1: Extract Serial Interface Methods](#proposal-1)
2. [Proposal #2: Extract Constants](#proposal-2)
3. [Proposal #3: Extract OdometryPublisher Class](#proposal-3)
4. [Proposal #4: Extract DiagnosticsPublisher Class](#proposal-4)
5. [Implementation Order](#implementation-order)

---

## Proposal #1: Extract Serial Interface Methods {#proposal-1}

### Problem

The `read_feedback()` method in `serial_interface.py` is **73 lines long** (lines 172-250) and mixes multiple responsibilities:

1. Buffer management
2. Frame synchronization
3. Packet unpacking
4. Checksum validation
5. Statistics updates

This makes it:
- Hard to understand at a glance
- Difficult to test individual steps
- Challenging to debug sync issues

### Current Code

```python
# serial_interface.py, lines 172-250
def read_feedback(self) -> Optional[HoverboardFeedback]:
    """
    Read telemetry feedback from hoverboard.

    Feedback is sent at 100Hz by firmware. This is a non-blocking read.
    Uses byte-by-byte scanning to find start frame and synchronize.

    Returns:
        HoverboardFeedback object if valid packet received, None otherwise
    """
    if not self.is_connected():
        return None

    try:
        # Read available bytes into sync buffer
        available = self.serial.in_waiting
        if available > 0:
            new_bytes = self.serial.read(available)
            self.sync_buffer.extend(new_bytes)

        # Keep buffer manageable
        if len(self.sync_buffer) > 200:
            self.sync_buffer = self.sync_buffer[-100:]

        # Need at least 18 bytes to check for a packet
        if len(self.sync_buffer) < self.FEEDBACK_PACKET_SIZE:
            return None

        # Scan for start frame (0xABCD in little-endian = 0xCD 0xAB)
        for i in range(len(self.sync_buffer) - self.FEEDBACK_PACKET_SIZE + 1):
            # Check if we found start frame
            if self.sync_buffer[i] == 0xCD and self.sync_buffer[i+1] == 0xAB:
                # Extract potential packet
                packet_data = bytes(self.sync_buffer[i:i+self.FEEDBACK_PACKET_SIZE])

                # Try to unpack and validate
                try:
                    unpacked = struct.unpack('<HhhhhhhHH', packet_data)
                    start, cmd1, cmd2, speed_r, speed_l, bat_v, temp, led, rx_checksum = unpacked

                    # Validate start frame (redundant but safe)
                    if start != self.START_FRAME:
                        continue

                    # Validate checksum
                    calc_checksum = start
                    for value in unpacked[1:-1]:
                        calc_checksum ^= (value & 0xFFFF)
                    calc_checksum &= 0xFFFF

                    if calc_checksum == rx_checksum:
                        # Valid packet! Remove from buffer including this packet
                        self.sync_buffer = self.sync_buffer[i+self.FEEDBACK_PACKET_SIZE:]
                        self.rx_count += 1

                        return HoverboardFeedback(
                            cmd1=cmd1,
                            cmd2=cmd2,
                            speed_r_rpm=speed_r,
                            speed_l_rpm=speed_l,
                            bat_voltage=bat_v,
                            board_temp=temp,
                            led=led,
                            timestamp=time.time()
                        )
                    else:
                        # Checksum failed - keep searching
                        self.checksum_errors += 1

                except struct.error:
                    # Unpacking failed - keep searching
                    self.framing_errors += 1

        # No valid packet found in buffer
        # Remove processed bytes but keep last 17 in case start frame spans boundary
        if len(self.sync_buffer) > 17:
            self.sync_buffer = self.sync_buffer[-(self.FEEDBACK_PACKET_SIZE-1):]

        return None

    except serial.SerialException as e:
        print(f"Serial read error: {e}")
        return None
```

### Proposed Refactored Code

```python
# serial_interface.py - refactored

# Add constants at module level
MAX_SYNC_BUFFER_SIZE = 200
SYNC_BUFFER_KEEP_SIZE = 100
SYNC_BUFFER_TAIL_SIZE = 17  # FEEDBACK_PACKET_SIZE - 1


def read_feedback(self) -> Optional[HoverboardFeedback]:
    """
    Read telemetry feedback from hoverboard.

    Non-blocking read that scans for valid packets in sync buffer.
    Uses byte-by-byte scanning to find start frame (0xCD 0xAB).

    Returns:
        HoverboardFeedback object if valid packet received, None otherwise
    """
    if not self.is_connected():
        return None

    try:
        self._refill_sync_buffer()
        self._trim_sync_buffer()

        if len(self.sync_buffer) < self.FEEDBACK_PACKET_SIZE:
            return None

        return self._scan_for_valid_packet()

    except serial.SerialException as e:
        print(f"Serial read error: {e}")
        return None


def _refill_sync_buffer(self) -> None:
    """Read available bytes from serial port into sync buffer."""
    available = self.serial.in_waiting
    if available > 0:
        new_bytes = self.serial.read(available)
        self.sync_buffer.extend(new_bytes)


def _trim_sync_buffer(self) -> None:
    """Keep sync buffer size manageable to prevent memory bloat."""
    if len(self.sync_buffer) > MAX_SYNC_BUFFER_SIZE:
        self.sync_buffer = self.sync_buffer[-SYNC_BUFFER_KEEP_SIZE:]


def _scan_for_valid_packet(self) -> Optional[HoverboardFeedback]:
    """
    Scan sync buffer for start frame and attempt to parse packet.

    Searches byte-by-byte for start frame (0xCD 0xAB) and attempts
    to parse and validate any potential packets found.

    Returns:
        HoverboardFeedback if valid packet found, None otherwise
    """
    # Scan for start frame (0xABCD in little-endian = 0xCD 0xAB)
    scan_range = len(self.sync_buffer) - self.FEEDBACK_PACKET_SIZE + 1

    for i in range(scan_range):
        if self._is_start_frame_at(i):
            feedback = self._try_parse_packet_at(i)
            if feedback is not None:
                # Valid packet found - remove it from buffer
                self.sync_buffer = self.sync_buffer[i + self.FEEDBACK_PACKET_SIZE:]
                self.rx_count += 1
                return feedback

    # No valid packet found - trim buffer but keep tail for boundary cases
    self._trim_buffer_tail()
    return None


def _is_start_frame_at(self, index: int) -> bool:
    """
    Check if start frame exists at buffer index.

    Args:
        index: Buffer index to check

    Returns:
        True if start frame (0xCD 0xAB) found at index
    """
    return (self.sync_buffer[index] == 0xCD and
            self.sync_buffer[index + 1] == 0xAB)


def _try_parse_packet_at(self, index: int) -> Optional[HoverboardFeedback]:
    """
    Attempt to parse and validate packet at buffer index.

    Args:
        index: Buffer index where potential packet starts

    Returns:
        HoverboardFeedback if packet valid, None otherwise
    """
    packet_data = bytes(self.sync_buffer[index:index + self.FEEDBACK_PACKET_SIZE])

    try:
        unpacked = struct.unpack('<HhhhhhhHH', packet_data)
        start, cmd1, cmd2, speed_r, speed_l, bat_v, temp, led, rx_checksum = unpacked

        # Validate start frame (redundant but safe)
        if start != self.START_FRAME:
            return None

        # Validate checksum
        if not self._validate_checksum(unpacked, rx_checksum):
            self.checksum_errors += 1
            return None

        # Valid packet!
        return HoverboardFeedback(
            cmd1=cmd1,
            cmd2=cmd2,
            speed_r_rpm=speed_r,
            speed_l_rpm=speed_l,
            bat_voltage=bat_v,
            board_temp=temp,
            led=led,
            timestamp=time.time()
        )

    except struct.error:
        # Unpacking failed
        self.framing_errors += 1
        return None


def _validate_checksum(self, unpacked: tuple, rx_checksum: int) -> bool:
    """
    Validate packet checksum.

    Args:
        unpacked: Tuple of unpacked packet values
        rx_checksum: Received checksum from packet

    Returns:
        True if checksum valid, False otherwise
    """
    calc_checksum = unpacked[0]  # Start with START_FRAME
    for value in unpacked[1:-1]:  # XOR all fields except checksum
        calc_checksum ^= (value & 0xFFFF)
    calc_checksum &= 0xFFFF

    return calc_checksum == rx_checksum


def _trim_buffer_tail(self) -> None:
    """
    Trim processed bytes from buffer, keeping tail for boundary cases.

    Keeps last 17 bytes in case start frame spans chunk boundary.
    """
    if len(self.sync_buffer) > SYNC_BUFFER_TAIL_SIZE:
        self.sync_buffer = self.sync_buffer[-SYNC_BUFFER_TAIL_SIZE:]
```

### Benefits

✅ **Readability**: Main `read_feedback()` method now fits on one screen
✅ **Testability**: Each step can be unit tested independently
✅ **Debuggability**: Easier to add logging to specific steps
✅ **Maintainability**: Clear what each method does
✅ **No logic changes**: Pure extraction, same behavior

### Risks

**Risk Level: LOW**

- Pure method extraction, no algorithmic changes
- Frame synchronization logic preserved exactly
- Easy to revert if issues arise

### Testing Strategy

1. **Unit tests** for new private methods:
   ```python
   def test_is_start_frame_at():
       """Test start frame detection."""
       interface = HoverboardSerialInterface()
       interface.sync_buffer = bytearray([0x00, 0xCD, 0xAB, 0x00])
       assert interface._is_start_frame_at(1) == True
       assert interface._is_start_frame_at(0) == False

   def test_validate_checksum():
       """Test checksum validation."""
       # Create unpacked tuple with known checksum
       # Verify validation works
   ```

2. **Integration test** with real hardware:
   - Run for 10 minutes
   - Verify identical RX counts before/after
   - Check for zero checksum/framing errors
   - Compare diagnostics output

3. **Regression test**:
   - Record telemetry data to file
   - Replay through both old and new code
   - Verify identical parsing results

### Implementation Steps

1. Add constants at module level
2. Extract `_refill_sync_buffer()` method
3. Extract `_trim_sync_buffer()` method
4. Extract `_is_start_frame_at()` method
5. Extract `_validate_checksum()` method
6. Extract `_try_parse_packet_at()` method
7. Extract `_scan_for_valid_packet()` method
8. Extract `_trim_buffer_tail()` method
9. Refactor `read_feedback()` to call extracted methods
10. Test with hardware

**Estimated Time:** 4 hours (including testing)

---

## Proposal #2: Extract Constants {#proposal-2}

### Problem

Magic numbers scattered throughout the codebase make it:
- Harder to understand what values mean
- Difficult to tune parameters
- Error-prone (typos not caught)

### Changes Required

#### File: `serial_interface.py`

```python
# Add at module level (after imports)

# Protocol constants
START_FRAME = 0xABCD
CMD_PACKET_SIZE = 8   # bytes
FEEDBACK_PACKET_SIZE = 18  # bytes

# Command limits
CMD_MIN = -1000
CMD_MAX = 1000

# Buffer management
MAX_SYNC_BUFFER_SIZE = 200
SYNC_BUFFER_KEEP_SIZE = 100
SYNC_BUFFER_TAIL_SIZE = 17  # FEEDBACK_PACKET_SIZE - 1

# Serial timeout
DEFAULT_TIMEOUT = 0.1  # seconds

# Checksum constants
UINT16_MASK = 0xFFFF
```

#### File: `hoverbot_driver_node.py`

```python
# Add at module level (after imports)

# Control loop timing
CONTROL_LOOP_HZ = 50  # Required by firmware for heartbeat
CONTROL_LOOP_PERIOD = 1.0 / CONTROL_LOOP_HZ  # 0.02 seconds

# Default parameters
DEFAULT_SERIAL_PORT = '/dev/ttyAMA0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_WHEEL_DIAMETER = 0.165  # meters
DEFAULT_WHEELBASE = 0.40  # meters
DEFAULT_MAX_RPM = 300
DEFAULT_CMD_TIMEOUT = 0.5  # seconds
DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_BASE_FRAME = 'base_link'

# Diagnostic publishing
DIAGNOSTICS_DECIMATION = 10  # Publish every Nth control loop (5Hz)

# Covariance values (rough estimates - tune based on testing)
POSE_COVARIANCE_X = 0.01    # meters²
POSE_COVARIANCE_Y = 0.01    # meters²
POSE_COVARIANCE_YAW = 0.05  # radians²
TWIST_COVARIANCE_VX = 0.01  # (m/s)²
TWIST_COVARIANCE_VYAW = 0.05  # (rad/s)²

# Feedback retry
MAX_FEEDBACK_RETRIES = 3
FEEDBACK_RETRY_DELAY = 0.001  # seconds
```

**Usage examples:**

```python
# Before
self.timer = self.create_timer(0.02, self.control_loop)

# After
self.timer = self.create_timer(CONTROL_LOOP_PERIOD, self.control_loop)

# Before
if self.serial.rx_count % 10 == 0:

# After
if self.serial.rx_count % DIAGNOSTICS_DECIMATION == 0:

# Before
odom.pose.covariance[0] = 0.01  # x

# After
odom.pose.covariance[0] = POSE_COVARIANCE_X
```

#### File: `differential_drive_controller.py`

```python
# Add at module level

# Unit conversions
SECONDS_PER_MINUTE = 60.0
RADIANS_PER_ROTATION = 2.0 * math.pi

# Default parameters
DEFAULT_WHEEL_DIAMETER = 0.165  # meters
DEFAULT_WHEELBASE = 0.40  # meters
DEFAULT_MAX_RPM = 300
DEFAULT_CMD_RANGE = 1000  # firmware command range
```

**Usage examples:**

```python
# Before
self.max_wheel_speed = (self.max_rpm * 2.0 * math.pi * self.wheel_radius) / 60.0

# After
self.max_wheel_speed = (
    self.max_rpm * RADIANS_PER_ROTATION * self.wheel_radius
) / SECONDS_PER_MINUTE

# Before
rpm_left = (v_left * 60.0) / (2.0 * math.pi * self.wheel_radius)

# After
rpm_left = (v_left * SECONDS_PER_MINUTE) / (RADIANS_PER_ROTATION * self.wheel_radius)
```

### Benefits

✅ **Self-documenting**: Names explain what values mean
✅ **Easier to tune**: Change in one place
✅ **Type safety**: Can add type hints to constants
✅ **No magic numbers**: Every value has meaning

### Risks

**Risk Level: VERY LOW**

- No logic changes
- Just naming existing values
- Easy to verify correctness

### Testing Strategy

1. No new tests needed
2. Existing tests should pass unchanged
3. Visual code review to verify values match

**Estimated Time:** 2 hours

---

## Proposal #3: Extract OdometryPublisher Class {#proposal-3}

### Problem

`HoverBotDriverNode` handles multiple responsibilities:
- Command processing
- **Odometry calculation** ← could be separate
- **Odometry publishing** ← could be separate
- **TF broadcasting** ← could be separate
- Diagnostics publishing

This violates Single Responsibility Principle.

### Proposed New File: `odometry_publisher.py`

```python
"""
Odometry Publisher

Manages robot pose estimation and publishing for differential drive robot.
Uses dead reckoning from wheel encoder feedback.
"""

import math
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from .differential_drive_controller import DifferentialDriveController


# Covariance values
POSE_COVARIANCE_X = 0.01
POSE_COVARIANCE_Y = 0.01
POSE_COVARIANCE_YAW = 0.05
TWIST_COVARIANCE_VX = 0.01
TWIST_COVARIANCE_VYAW = 0.05


class OdometryPublisher:
    """
    Manages robot odometry estimation and publishing.

    Responsibilities:
    - Dead reckoning from wheel velocities
    - Odometry message publishing
    - TF transform broadcasting
    - Pose state management
    """

    def __init__(
        self,
        node: Node,
        controller: DifferentialDriveController,
        odom_frame: str = 'odom',
        base_frame: str = 'base_link',
        publish_odom: bool = True,
        publish_tf: bool = True
    ):
        """
        Initialize odometry publisher.

        Args:
            node: ROS2 node for creating publishers
            controller: Differential drive controller for kinematics
            odom_frame: Name of odometry frame
            base_frame: Name of robot base frame
            publish_odom: Whether to publish odometry messages
            publish_tf: Whether to broadcast TF transforms
        """
        self.node = node
        self.controller = controller
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.publish_odom_flag = publish_odom
        self.publish_tf_flag = publish_tf

        # Pose state (odometry frame)
        self.x = 0.0      # meters
        self.y = 0.0      # meters
        self.theta = 0.0  # radians

        # Timing
        self.last_time = node.get_clock().now()

        # Publishers
        if self.publish_odom_flag:
            self.odom_pub = node.create_publisher(Odometry, 'odom', 10)

        if self.publish_tf_flag:
            self.tf_broadcaster = TransformBroadcaster(node)

    def update_from_wheels(self, rpm_left: float, rpm_right: float) -> None:
        """
        Update robot pose from wheel encoder feedback.

        Uses differential drive kinematics and Euler integration
        for dead reckoning.

        Args:
            rpm_left: Left wheel speed in RPM
            rpm_right: Right wheel speed in RPM (sign-corrected)
        """
        # Calculate time delta
        current_time = self.node.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Convert wheel speeds to robot velocities
        linear_x, angular_z = self.controller.wheels_to_twist(rpm_left, rpm_right)

        # Integrate pose
        self._integrate_pose(linear_x, angular_z, dt)

        # Publish
        if self.publish_odom_flag:
            self._publish_odometry(current_time, linear_x, angular_z)

        if self.publish_tf_flag:
            self._publish_transform(current_time)

    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """
        Reset robot pose to specified values.

        Args:
            x: X position in meters
            y: Y position in meters
            theta: Heading in radians
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.last_time = self.node.get_clock().now()

    def get_pose(self) -> tuple:
        """
        Get current robot pose.

        Returns:
            Tuple of (x, y, theta)
        """
        return (self.x, self.y, self.theta)

    def _integrate_pose(self, linear_x: float, angular_z: float, dt: float) -> None:
        """
        Integrate velocities to update pose estimate.

        Uses Euler integration with midpoint method for better accuracy.

        Args:
            linear_x: Linear velocity in m/s
            angular_z: Angular velocity in rad/s
            dt: Time delta in seconds
        """
        # Euler integration with midpoint method
        # This reduces error for curved trajectories
        delta_theta = angular_z * dt
        delta_x = linear_x * math.cos(self.theta + delta_theta / 2.0) * dt
        delta_y = linear_x * math.sin(self.theta + delta_theta / 2.0) * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def _publish_odometry(self, timestamp, linear_x: float, angular_z: float) -> None:
        """
        Publish odometry message.

        Args:
            timestamp: ROS time for message
            linear_x: Current linear velocity (m/s)
            angular_z: Current angular velocity (rad/s)
        """
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation = self._quaternion_from_yaw(self.theta)

        # Velocity
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_z

        # Covariance
        odom.pose.covariance[0] = POSE_COVARIANCE_X   # x
        odom.pose.covariance[7] = POSE_COVARIANCE_Y   # y
        odom.pose.covariance[35] = POSE_COVARIANCE_YAW  # yaw

        odom.twist.covariance[0] = TWIST_COVARIANCE_VX   # vx
        odom.twist.covariance[35] = TWIST_COVARIANCE_VYAW  # vyaw

        self.odom_pub.publish(odom)

    def _publish_transform(self, timestamp) -> None:
        """
        Broadcast TF transform from odom to base_link.

        Args:
            timestamp: ROS time for transform
        """
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # Translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Rotation
        t.transform.rotation = self._quaternion_from_yaw(self.theta)

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        """
        Convert yaw angle to quaternion.

        Args:
            yaw: Rotation around Z-axis in radians

        Returns:
            Quaternion message
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
```

### Updated `hoverbot_driver_node.py`

```python
# In imports, add:
from .odometry_publisher import OdometryPublisher

# In __init__(), replace odometry setup with:
self.odometry = OdometryPublisher(
    node=self,
    controller=self.controller,
    odom_frame=self.odom_frame,
    base_frame=self.base_frame,
    publish_odom=self.publish_odom_flag,
    publish_tf=self.publish_tf_flag
)

# REMOVE these from __init__:
# - self.x, self.y, self.theta
# - self.last_odom_time
# - self.odom_pub
# - self.tf_broadcaster

# In control_loop(), replace odometry update:
if feedback is not None:
    # NOTE: Right wheel RPM negation is HARDWARE QUIRK - must stay here
    self.odometry.update_from_wheels(feedback.speed_l_rpm, -feedback.speed_r_rpm)

    # Publish diagnostics every 10 cycles (5Hz)
    if self.serial.rx_count % DIAGNOSTICS_DECIMATION == 0:
        self.publish_diagnostics(feedback)

# REMOVE these methods entirely:
# - update_odometry()
# - publish_odometry()
# - publish_transform()
# - quaternion_from_yaw()
```

### Benefits

✅ **Single Responsibility**: Each class has one reason to change
✅ **Testability**: Can unit test odometry independently
✅ **Reusability**: Could use OdometryPublisher in other robots
✅ **Clarity**: Main driver node is now just an orchestrator
✅ **Extensibility**: Easy to add IMU fusion later

### Risks

**Risk Level: MEDIUM**

- More significant architectural change
- Must preserve exact odometry math
- Must maintain timing (50Hz updates)
- Right wheel sign correction must stay in driver node

### Testing Strategy

1. **Unit tests** for OdometryPublisher:
   ```python
   def test_straight_line_motion():
       """Test pose integration for straight line."""
       # Create mock node and controller
       # Update with equal wheel speeds
       # Verify x increases, y=0, theta=0

   def test_rotation_in_place():
       """Test pose integration for rotation."""
       # Update with opposite wheel speeds
       # Verify x=0, y=0, theta changes

   def test_reset_pose():
       """Test pose reset functionality."""
       # Move robot
       # Reset pose
       # Verify state reset
   ```

2. **Integration test** with hardware:
   - Drive straight for 1 meter
   - Record odometry before/after refactor
   - Compare outputs (should be identical within floating point error)
   - Check TF tree structure unchanged

3. **Visual test** in RViz:
   - Drive robot in square pattern
   - Verify odometry path looks identical

**Estimated Time:** 6 hours (including thorough testing)

---

## Proposal #4: Extract DiagnosticsPublisher Class {#proposal-4}

### Problem

Diagnostics publishing is another distinct responsibility mixed into the main driver node.

### Proposed New File: `diagnostics_publisher.py`

```python
"""
Diagnostics Publisher

Publishes robot health and status diagnostics.
"""

from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .serial_interface import HoverboardFeedback


# Diagnostic thresholds
LOW_BATTERY_VOLTAGE = 30.0  # Volts
HIGH_TEMPERATURE = 60.0     # Celsius

# Unit conversions
BATTERY_SCALE = 100.0       # Raw value × 100
TEMPERATURE_SCALE = 10.0    # Raw value × 10


class DiagnosticsPublisher:
    """
    Publishes robot diagnostics.

    Responsibilities:
    - Battery voltage monitoring
    - Temperature monitoring
    - Communication statistics
    - Status level determination
    """

    def __init__(self, node: Node, hardware_id: str = "hoverboard_uart"):
        """
        Initialize diagnostics publisher.

        Args:
            node: ROS2 node for creating publisher
            hardware_id: Hardware identifier for diagnostics
        """
        self.node = node
        self.hardware_id = hardware_id
        self.diag_pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10)

    def publish(self, feedback: HoverboardFeedback, serial_stats: dict) -> None:
        """
        Publish diagnostic information.

        Args:
            feedback: Hoverboard feedback data
            serial_stats: Serial communication statistics
        """
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.node.get_clock().now().to_msg()

        # Create hoverboard status
        status = self._create_hoverboard_status(feedback, serial_stats)
        diag_array.status.append(status)

        self.diag_pub.publish(diag_array)

    def _create_hoverboard_status(
        self,
        feedback: HoverboardFeedback,
        serial_stats: dict
    ) -> DiagnosticStatus:
        """
        Create diagnostic status for hoverboard controller.

        Args:
            feedback: Hoverboard telemetry data
            serial_stats: Communication statistics

        Returns:
            DiagnosticStatus message
        """
        status = DiagnosticStatus()
        status.name = "HoverBoard Controller"
        status.hardware_id = self.hardware_id

        # Convert raw values to physical units
        battery_v = feedback.bat_voltage / BATTERY_SCALE
        temp_c = feedback.board_temp / TEMPERATURE_SCALE

        # Determine status level and message
        status.level, status.message = self._determine_status_level(battery_v, temp_c)

        # Add diagnostic values
        status.values = self._create_diagnostic_values(
            feedback, serial_stats, battery_v, temp_c
        )

        return status

    def _determine_status_level(self, battery_v: float, temp_c: float) -> tuple:
        """
        Determine diagnostic status level based on battery and temperature.

        Args:
            battery_v: Battery voltage in volts
            temp_c: Board temperature in Celsius

        Returns:
            Tuple of (status_level, message)
        """
        if battery_v < LOW_BATTERY_VOLTAGE:
            return (DiagnosticStatus.WARN, "Low battery voltage")
        elif temp_c > HIGH_TEMPERATURE:
            return (DiagnosticStatus.WARN, "High temperature")
        else:
            return (DiagnosticStatus.OK, "Operating normally")

    def _create_diagnostic_values(
        self,
        feedback: HoverboardFeedback,
        stats: dict,
        battery_v: float,
        temp_c: float
    ) -> list:
        """
        Create list of diagnostic key-value pairs.

        Args:
            feedback: Hoverboard telemetry data
            stats: Communication statistics
            battery_v: Battery voltage in volts
            temp_c: Temperature in Celsius

        Returns:
            List of KeyValue messages
        """
        return [
            KeyValue(key="Battery Voltage", value=f"{battery_v:.2f} V"),
            KeyValue(key="Temperature", value=f"{temp_c:.1f} °C"),
            KeyValue(key="Speed Left", value=f"{feedback.speed_l_rpm} RPM"),
            KeyValue(key="Speed Right", value=f"{feedback.speed_r_rpm} RPM"),
            KeyValue(key="TX Count", value=str(stats['tx_count'])),
            KeyValue(key="RX Count", value=str(stats['rx_count'])),
            KeyValue(key="Checksum Errors", value=str(stats['checksum_errors'])),
            KeyValue(key="Framing Errors", value=str(stats['framing_errors']))
        ]
```

### Updated `hoverbot_driver_node.py`

```python
# In imports:
from .diagnostics_publisher import DiagnosticsPublisher

# In __init__():
self.diagnostics = DiagnosticsPublisher(node=self)

# In control_loop():
if feedback is not None:
    # Update odometry
    self.odometry.update_from_wheels(feedback.speed_l_rpm, -feedback.speed_r_rpm)

    # Publish diagnostics every 10 cycles (5Hz)
    if self.serial.rx_count % DIAGNOSTICS_DECIMATION == 0:
        stats = self.serial.get_stats()
        self.diagnostics.publish(feedback, stats)

# REMOVE publish_diagnostics() method entirely
```

### Benefits

✅ **Separation of Concerns**: Diagnostics logic isolated
✅ **Testability**: Easy to unit test status determination
✅ **Extensibility**: Easy to add more diagnostic checks
✅ **Constants**: Magic numbers moved to named constants
✅ **Clarity**: Status level logic is explicit

### Risks

**Risk Level: LOW**

- Simple extraction
- No complex state management
- Easy to verify output is identical

### Testing Strategy

1. **Unit tests**:
   ```python
   def test_low_battery_warning():
       """Test low battery triggers warning status."""
       # Create feedback with low battery
       # Verify status level is WARN

   def test_high_temperature_warning():
       """Test high temp triggers warning."""
       # Create feedback with high temp
       # Verify status level is WARN

   def test_normal_operation():
       """Test normal values trigger OK status."""
       # Create feedback with normal values
       # Verify status level is OK
   ```

2. **Integration test**:
   - Subscribe to /diagnostics topic
   - Compare message contents before/after
   - Verify identical output

**Estimated Time:** 3 hours

---

## Implementation Order {#implementation-order}

### Recommended Sequence

```
Phase 1: Foundation (Week 1)
├── 1. Extract Constants (Proposal #2)
│   └── All files, ~2 hours, VERY LOW risk
├── 2. Add Type Hints
│   └── All files, ~1 hour, VERY LOW risk
└── 3. Enhance Docstrings
    └── All files, ~2 hours, NO risk

Phase 2: Serial Interface (Week 2)
├── 4. Extract Serial Methods (Proposal #1)
│   └── serial_interface.py, ~4 hours, LOW risk
└── 5. Test Serial Interface
    └── Unit + integration tests, ~4 hours, NO risk

Phase 3: Driver Node (Week 3-4)
├── 6. Extract DiagnosticsPublisher (Proposal #4)
│   └── New file, ~3 hours, LOW risk
├── 7. Extract OdometryPublisher (Proposal #3)
│   └── New file, ~6 hours, MEDIUM risk
└── 8. Test Driver Node
    └── Unit + integration tests, ~6 hours, NO risk

Phase 4: Documentation (Week 5)
└── 9. Create Architecture Docs
    └── ARCHITECTURE.md, CONTRIBUTING.md, etc.
```

### Success Criteria for Each Phase

**Phase 1:**
- [ ] All magic numbers replaced with named constants
- [ ] All methods have type hints
- [ ] All methods have comprehensive docstrings
- [ ] Code review passes
- [ ] No functional changes

**Phase 2:**
- [ ] `read_feedback()` method < 15 lines
- [ ] All extracted methods have unit tests
- [ ] Integration test with hardware passes
- [ ] Zero regressions in packet parsing
- [ ] Checksum/framing error counts unchanged

**Phase 3:**
- [ ] `HoverBotDriverNode` < 250 lines
- [ ] Odometry publishing isolated in separate class
- [ ] Diagnostics publishing isolated in separate class
- [ ] All new classes have unit tests
- [ ] Integration test with hardware passes
- [ ] Odometry output identical to before
- [ ] Diagnostics output identical to before
- [ ] TF tree structure unchanged
- [ ] Robot drives identically

**Phase 4:**
- [ ] ARCHITECTURE.md explains design decisions
- [ ] CONTRIBUTING.md has coding standards
- [ ] New team member can understand codebase
- [ ] Documentation is up to date

---

## Critical Reminders

### DO NOT MODIFY (without explicit discussion):

1. ⚠️ **Frame sync algorithm** - Lines 200-248 in serial_interface.py
2. ⚠️ **Right wheel sign correction** - Line 196 in hoverbot_driver_node.py
3. ⚠️ **50Hz timing** - Line 124 in hoverbot_driver_node.py
4. ⚠️ **Checksum calculation** - Lines 118-133, 217-220 in serial_interface.py
5. ⚠️ **Kinematics equations** - differential_drive_controller.py

### SAFE TO MODIFY:

- ✅ Method extraction (preserving logic)
- ✅ Class organization
- ✅ Constants and configuration
- ✅ Documentation and comments
- ✅ Type hints
- ✅ Tests
- ✅ Error handling
- ✅ Logging

---

## Questions Before Proceeding?

Before implementing any of these proposals, please confirm:

1. **Which proposal would you like to start with?**
   - Proposal #2 (Constants) is lowest risk
   - Proposal #1 (Serial methods) has highest impact
   - Proposal #3 (Odometry) is biggest architectural change

2. **Testing approach:**
   - Do you have hardware available for integration testing?
   - Should I create unit tests before refactoring?
   - What's your preferred test framework? (pytest, unittest, ROS2 test)

3. **Timeline:**
   - Is the phased approach (4-5 weeks) acceptable?
   - Or should we focus on just one proposal?

4. **Git workflow:**
   - One branch per proposal?
   - Or one branch for all refactorings?

---

**Ready to proceed when you are!** 🚀
