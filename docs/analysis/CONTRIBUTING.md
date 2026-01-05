# Contributing to HoverBot

Thank you for your interest in contributing to the HoverBot project! This document provides guidelines and standards for contributing code, documentation, and improvements.

---

## Table of Contents

1. [Getting Started](#getting-started)
2. [Coding Standards](#coding-standards)
3. [Testing Requirements](#testing-requirements)
4. [Pull Request Process](#pull-request-process)
5. [Code Review Checklist](#code-review-checklist)
6. [Design Philosophy](#design-philosophy)

---

## Getting Started

### Development Environment Setup

1. **Fork and Clone**
   ```bash
   git clone https://github.com/YOUR_USERNAME/hoverbot.git
   cd hoverbot
   ```

2. **Create Development Branch**
   ```bash
   git checkout -b feature/your-feature-name
   # or
   git checkout -b fix/issue-description
   ```

3. **Install Dependencies**
   ```bash
   cd ~/hoverbot/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build**
   ```bash
   colcon build --packages-select hoverbot_driver
   source install/setup.bash
   ```

### Development Workflow

1. **Make changes** in `src/hoverbot_driver/`
2. **Test locally** with `test/test_serial_protocol.py`
3. **Rebuild** with `colcon build --packages-select hoverbot_driver`
4. **Test with hardware** (if available)
5. **Commit** with descriptive messages
6. **Push** and create pull request

---

## Coding Standards

### Python Style Guide

We follow **PEP 8** with some project-specific conventions.

#### General Rules

```python
# ✅ GOOD: Clear, descriptive names
def calculate_wheel_velocities(linear_x: float, angular_z: float) -> Tuple[float, float]:
    """Calculate individual wheel velocities from robot twist."""
    pass

# ❌ BAD: Unclear abbreviations, missing types
def calc_vel(x, z):
    pass
```

#### Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Classes | PascalCase | `HoverBotDriverNode` |
| Functions/Methods | snake_case | `read_feedback()` |
| Constants | UPPER_SNAKE_CASE | `MAX_RPM`, `CONTROL_LOOP_HZ` |
| Private methods | _leading_underscore | `_validate_checksum()` |
| Variables | snake_case | `battery_voltage`, `wheel_diameter` |

#### Type Hints

**Always use type hints** for function signatures:

```python
# ✅ GOOD: Clear types
def twist_to_wheels(
    self,
    linear_x: float,
    angular_z: float
) -> Tuple[int, int]:
    pass

# ❌ BAD: No type information
def twist_to_wheels(self, linear_x, angular_z):
    pass
```

For complex types, import from `typing`:
```python
from typing import Optional, Tuple, Dict, List
```

#### Docstrings

Use **Google-style docstrings** for all public functions and classes:

```python
def send_command(self, steer: int, speed: int) -> bool:
    """
    Send motor command to hoverboard.

    With TANK_STEERING enabled:
    - steer: Left wheel command (-1000 to +1000)
    - speed: Right wheel command (-1000 to +1000)

    Args:
        steer: Left wheel command
        speed: Right wheel command

    Returns:
        True if transmission successful, False otherwise

    Raises:
        SerialException: If serial port is not open
    """
    pass
```

**Minimum required:**
- Brief description (one line)
- Args section (if parameters exist)
- Returns section (if not None)

#### Constants

**Extract magic numbers to named constants:**

```python
# ✅ GOOD: Self-documenting
CONTROL_LOOP_HZ = 50  # Required by firmware for heartbeat
CONTROL_LOOP_PERIOD = 1.0 / CONTROL_LOOP_HZ

self.timer = self.create_timer(CONTROL_LOOP_PERIOD, self.control_loop)

# ❌ BAD: Magic number
self.timer = self.create_timer(0.02, self.control_loop)  # What is 0.02?
```

Place constants at **module level** (after imports).

#### Imports

Organize imports in this order:
1. Standard library
2. Third-party libraries
3. ROS2 libraries
4. Local modules

```python
# Standard library
import math
import time
from typing import Optional, Tuple

# Third-party
import serial

# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Local
from .serial_interface import HoverboardSerialInterface
from .differential_drive_controller import DifferentialDriveController
```

Use `isort` to auto-format:
```bash
isort hoverbot_driver_node.py
```

#### Line Length

- **Target:** 88 characters (Black formatter default)
- **Maximum:** 100 characters (hard limit)
- Break long lines logically:

```python
# ✅ GOOD: Logical breaks
status.values = [
    KeyValue(key="Battery Voltage", value=f"{battery_v:.2f} V"),
    KeyValue(key="Temperature", value=f"{temp_c:.1f} °C"),
    KeyValue(key="Speed Left", value=f"{feedback.speed_l_rpm} RPM"),
]

# ❌ BAD: Exceeds 100 characters
status.values = [KeyValue(key="Battery Voltage", value=f"{battery_v:.2f} V"), KeyValue(key="Temperature", value=f"{temp_c:.1f} °C")]
```

### Sandi Metz Rules (Guidelines)

We strive to follow **Sandi Metz's design principles**:

1. **Classes < 100 lines**
   - If a class exceeds 100 lines, consider splitting responsibilities
   - Exceptions allowed for ROS2 nodes (boilerplate is verbose)

2. **Methods < 5 lines**
   - Strive for small, focused methods
   - Extract complex logic into helper methods
   - Exceptions allowed for message construction (ROS2 messages are verbose)

3. **Pass < 4 parameters**
   - Use dataclasses or config objects for groups of related parameters
   - Consider builder pattern for complex initialization

4. **Controllers instantiate one object**
   - Main classes should delegate to specialized components
   - Avoid "God objects" that do everything

**Remember:** These are guidelines, not laws. Break them when you have good reason and can explain why.

### Comments

**Use comments to explain WHY, not WHAT:**

```python
# ✅ GOOD: Explains rationale
# Negate right wheel RPM because firmware reports it backwards
# This is a hardware quirk, not a protocol issue
self.update_odometry(feedback.speed_l_rpm, -feedback.speed_r_rpm)

# ❌ BAD: Redundant with code
# Negate right wheel
self.update_odometry(feedback.speed_l_rpm, -feedback.speed_r_rpm)
```

**Mark critical sections:**

```python
# ⚠️ CRITICAL: Frame sync algorithm - do not modify without hardware testing
# Byte-by-byte scanning is required because packets arrive at arbitrary boundaries
for i in range(len(self.sync_buffer) - self.FEEDBACK_PACKET_SIZE + 1):
    if self.sync_buffer[i] == 0xCD and self.sync_buffer[i+1] == 0xAB:
        # ...
```

### Error Handling

**Fail fast and loud:**

```python
# ✅ GOOD: Clear error messages
if not self.is_connected():
    self.get_logger().error("Cannot send command: Serial port not connected")
    return False

# ❌ BAD: Silent failure
if not self.is_connected():
    return False
```

**Use specific exceptions:**

```python
# ✅ GOOD: Specific exception
try:
    self.serial = serial.Serial(port=self.port, baudrate=self.baudrate)
except serial.SerialException as e:
    print(f"Failed to open serial port {self.port}: {e}")
    return False

# ❌ BAD: Catch all exceptions
except Exception as e:
    print("Error!")
    return False
```

---

## Testing Requirements

### Test Pyramid

```
          ┌────────────┐
          │ Integration│  ← Real hardware (manual)
          │   Tests    │
          └────────────┘
         ┌──────────────┐
         │     Unit      │  ← Pure functions (automated)
         │    Tests      │
         └──────────────┘
        ┌────────────────┐
        │   Standalone   │  ← Protocol validation
        │   Test Script  │
        └────────────────┘
```

### Unit Tests

**All new code should have unit tests.**

#### Test Structure

Use **pytest** for unit tests:

```python
# test/unit/test_differential_drive_controller.py
import pytest
from hoverbot_driver.differential_drive_controller import DifferentialDriveController


class TestDifferentialDriveController:
    """Unit tests for differential drive kinematics."""

    @pytest.fixture
    def controller(self):
        """Create controller with standard parameters."""
        return DifferentialDriveController(
            wheel_diameter=0.165,
            wheelbase=0.40,
            max_rpm=300
        )

    def test_zero_velocity(self, controller):
        """Test that zero velocity produces zero commands."""
        left, right = controller.twist_to_wheels(0.0, 0.0)
        assert left == 0
        assert right == 0

    def test_forward_motion(self, controller):
        """Test pure forward motion produces equal wheel commands."""
        left, right = controller.twist_to_wheels(0.2, 0.0)
        assert left == right
        assert left > 0

    def test_rotation_in_place(self, controller):
        """Test rotation produces opposite wheel commands."""
        left, right = controller.twist_to_wheels(0.0, 1.0)
        assert left == -right
```

#### What to Test

**Do test:**
- ✅ Pure functions (kinematics, conversions)
- ✅ Validation logic (checksum, limits)
- ✅ Edge cases (zero, max, negative values)
- ✅ Error handling (invalid input)

**Don't test:**
- ❌ ROS2 middleware (assume it works)
- ❌ Hardware (use integration tests)
- ❌ Third-party libraries (serial, struct)

#### Running Tests

```bash
# Run all tests
pytest test/

# Run specific test file
pytest test/unit/test_differential_drive_controller.py

# Run with coverage
pytest --cov=hoverbot_driver test/
```

### Integration Tests

**Test with real hardware when possible.**

Before submitting a PR that changes critical code:
1. Run `test/test_serial_protocol.py` - verify serial communication
2. Drive robot in RViz - verify odometry looks correct
3. Run for 10+ minutes - check for errors in `/diagnostics`
4. Verify RX/TX counts match before/after refactor

### Test-Driven Development (Recommended)

For new features:
1. **Write test first** (it will fail)
2. **Implement feature** (make test pass)
3. **Refactor** (keep test passing)

Example:
```python
# Step 1: Write failing test
def test_imu_fusion():
    """Test IMU data fuses with wheel odometry."""
    odometry = OdometryPublisher(...)
    odometry.update_from_wheels_and_imu(rpm_left, rpm_right, imu_data)
    # This will fail because method doesn't exist yet

# Step 2: Implement feature
def update_from_wheels_and_imu(self, rpm_left, rpm_right, imu_data):
    # Implementation here

# Step 3: Test passes, refactor if needed
```

---

## Pull Request Process

### Before Creating PR

**Checklist:**
- [ ] Code follows style guide (PEP 8, type hints, docstrings)
- [ ] Constants extracted (no magic numbers)
- [ ] Unit tests added for new code
- [ ] Integration test passed (if hardware available)
- [ ] Documentation updated (docstrings, README if needed)
- [ ] No compiler warnings
- [ ] Commit messages are descriptive

### PR Title Format

Use conventional commits:

```
type(scope): brief description

Examples:
feat(odometry): add IMU fusion support
fix(serial): handle timeout during frame sync
refactor(driver): extract OdometryPublisher class
docs(readme): update installation instructions
test(kinematics): add rotation validation tests
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `refactor`: Code restructuring (no behavior change)
- `docs`: Documentation only
- `test`: Adding/updating tests
- `perf`: Performance improvement
- `style`: Code style (formatting, typos)

### PR Description Template

```markdown
## Summary
Brief description of what this PR does.

## Motivation
Why is this change needed? What problem does it solve?

## Changes
- Bullet list of specific changes
- Use past tense ("Added", "Fixed", "Refactored")

## Testing
- [ ] Unit tests added/updated
- [ ] Integration test with hardware (describe results)
- [ ] Verified no regressions

## Checklist
- [ ] Code follows style guide
- [ ] Documentation updated
- [ ] Breaking changes documented (if any)

## Related Issues
Fixes #123
Related to #456
```

### Review Process

1. **Author** creates PR
2. **Reviewer** checks code (see checklist below)
3. **Author** addresses feedback
4. **Reviewer** approves
5. **Maintainer** merges

**Minimum reviewers:** 1 (for critical changes, 2)

---

## Code Review Checklist

### Functionality
- [ ] Code does what it claims to do
- [ ] Edge cases are handled
- [ ] No obvious bugs

### Design
- [ ] Follows Single Responsibility Principle
- [ ] Classes/methods have clear purpose
- [ ] No unnecessary complexity

### Safety
- [ ] No memory leaks (Python: circular references)
- [ ] Error handling is appropriate
- [ ] No race conditions (if multithreaded)

### Hardware-Specific (Critical!)
- [ ] Frame synchronization algorithm not modified (unless explicitly intended)
- [ ] Right wheel sign correction preserved
- [ ] 50Hz timing maintained
- [ ] Checksum calculation unchanged

### Testing
- [ ] Unit tests cover new code
- [ ] Tests are meaningful (not just for coverage)
- [ ] Integration test results documented

### Documentation
- [ ] Docstrings for public functions/classes
- [ ] Comments explain complex logic
- [ ] README updated if interface changed

### Style
- [ ] Follows PEP 8
- [ ] Type hints present
- [ ] No magic numbers
- [ ] Imports organized

---

## Design Philosophy

### Core Values

**1. Reliability Over Cleverness**

Prefer simple, obvious code over clever tricks:

```python
# ✅ GOOD: Clear and obvious
if battery_v < LOW_BATTERY_VOLTAGE:
    return (DiagnosticStatus.WARN, "Low battery voltage")
elif temp_c > HIGH_TEMPERATURE:
    return (DiagnosticStatus.WARN, "High temperature")
else:
    return (DiagnosticStatus.OK, "Operating normally")

# ❌ BAD: Clever but obscure
return (DiagnosticStatus.WARN, ["Low battery", "High temp"][int(temp_c > 60)]) \
    if battery_v < 30 or temp_c > 60 else (DiagnosticStatus.OK, "OK")
```

**2. Explicit Over Implicit**

Make intentions clear:

```python
# ✅ GOOD: Explicit hardware quirk
# Firmware reports right wheel backwards - negate for correct odometry
corrected_rpm_right = -feedback.speed_r_rpm
self.update_odometry(feedback.speed_l_rpm, corrected_rpm_right)

# ❌ BAD: Implicit negation
self.update_odometry(feedback.speed_l_rpm, -feedback.speed_r_rpm)
```

**3. Duplication Over Wrong Abstraction**

It's better to have duplicate code than the wrong abstraction:

```python
# ✅ ACCEPTABLE: Some duplication
def _publish_odometry(self, ...):
    odom = Odometry()
    odom.header.stamp = timestamp.to_msg()
    odom.header.frame_id = self.odom_frame
    # ...

def _publish_transform(self, ...):
    t = TransformStamped()
    t.header.stamp = timestamp.to_msg()
    t.header.frame_id = self.odom_frame
    # ...

# ❌ BAD: Premature abstraction
def _create_header(self, msg_type, timestamp):
    # Overly complex for 2 lines
```

**4. Change One Thing at a Time**

When refactoring:
- One PR = one logical change
- Don't mix refactoring with feature additions
- Don't mix style fixes with logic changes

**5. Preserve What Works**

The driver is **battle-tested**. If you're modifying critical sections:
- Explain why the change is necessary
- Test exhaustively with hardware
- Provide rollback plan

**Critical sections:**
- Frame synchronization (serial_interface.py)
- Odometry integration math (hoverbot_driver_node.py)
- Kinematics equations (differential_drive_controller.py)

---

## Questions?

- **General questions**: Open a GitHub issue
- **Design discussions**: See `ARCHITECTURE.md`
- **Refactoring plans**: See `REFACTORING_PROPOSALS.md`
- **Bugs/issues**: Search existing issues first, then create new one

---

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (MIT License).

---

**Thank you for contributing to HoverBot!** 🤖
