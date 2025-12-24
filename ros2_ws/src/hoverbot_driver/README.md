# HoverBot Driver - ROS 2 Package

ROS 2 driver for robots based on the [EFeru hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC). Provides serial communication, odometry estimation, and full integration with the ROS 2 navigation stack.

## Platform Support

**Compatible with:**
- ✅ **Ubuntu 22.04 LTS** + **ROS 2 Humble** (x86_64 or ARM64)
- ✅ **Ubuntu 24.04 LTS** + **ROS 2 Jazzy** (x86_64 or ARM64)
- ✅ **Raspberry Pi 4/5** (ARM64 architecture)

**Development Workflow:**
- Develop on Ubuntu 22.04 x86_64 (no hardware needed)
- Deploy to Raspberry Pi ARM64 (rebuild on target)

## Features

- ✅ **Bidirectional Serial Communication** - Commands and telemetry via UART
- ✅ **Differential Drive Kinematics** - Twist → wheel commands conversion
- ✅ **Odometry Publishing** - Dead reckoning from wheel encoders
- ✅ **TF Broadcasting** - odom → base_link transform
- ✅ **Diagnostic Monitoring** - Battery, temperature, communication stats
- ✅ **Heartbeat Management** - Prevents firmware timeout (50Hz)
- ✅ **Safety Features** - Command timeout, velocity limiting, error detection

## Hardware Requirements

- **Hoverboard Controller**: STM32F103 mainboard with EFeru FOC firmware
- **Firmware Configuration**:
  - `VARIANT_USART` enabled
  - `CONTROL_SERIAL_USART3` (right sideboard, 5V tolerant)
  - `TANK_STEERING` enabled
  - `BOARD_VARIANT = 1` (critical for power management)
  - Baud rate: 115200

- **Raspberry Pi**: UART connection to hoverboard
  - GPIO14 (TX) → Hoverboard RX
  - GPIO15 (RX) → Hoverboard TX
  - GND → GND

## Installation

### 1. Clone into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
cp -r /path/to/hoverbot_driver .
```

### 2. Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select hoverbot_driver
source install/setup.bash
```

## Quick Start

### Launch the driver

```bash
ros2 launch hoverbot_driver hoverbot_driver.launch.py
```

### Test with keyboard teleoperation

```bash
# Install teleop_twist_keyboard if needed
sudo apt install ros-jazzy-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You should see the wheels respond to keyboard commands and the robot begin moving.

## Configuration

Edit `config/hoverbot_driver.yaml` to customize parameters:

### Serial Settings
- `serial_port`: UART device path (default: `/dev/ttyAMA0`)
- `baud_rate`: Communication speed (default: `115200`)

### Robot Parameters
- `wheel_diameter`: Wheel diameter in meters (default: `0.165`)
- `wheelbase`: Track width in meters (default: `0.40`)
- `max_rpm`: Maximum wheel speed safety limit (default: `300`)

### Safety
- `cmd_vel_timeout`: Stop if no commands received (default: `0.5` seconds)

### Publishing
- `publish_odom`: Enable odometry publishing (default: `true`)
- `publish_tf`: Enable TF broadcasting (default: `true`)

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear.x, angular.z) |

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Robot pose and velocity |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 5 Hz | Battery, temperature, comm stats |

### TF Transforms

| Parent Frame | Child Frame | Description |
|--------------|-------------|-------------|
| `odom` | `base_link` | Robot pose in odometry frame |

## Protocol Details

### Command Packet (8 bytes, little-endian)
```
[0-1]  Start Frame: 0xABCD (uint16)
[2-3]  Left Wheel:  -1000 to +1000 (int16)
[4-5]  Right Wheel: -1000 to +1000 (int16)
[6-7]  Checksum:    XOR of all fields (uint16)
```

### Feedback Packet (18 bytes, little-endian)
```
[0-1]   Start Frame: 0xABCD
[2-3]   Cmd1: Processed left command
[4-5]   Cmd2: Processed right command
[6-7]   Speed Right: RPM (int16)
[8-9]   Speed Left:  RPM (int16)
[10-11] Battery:     Voltage × 100
[12-13] Temperature: °C × 10
[14-15] LED:         Control word (unused)
[16-17] Checksum:    XOR of all fields
```

## Kinematics

The driver implements differential drive kinematics for tank steering:

**Forward Kinematics** (wheel speeds → robot velocity):
```
v_robot = (v_left + v_right) / 2
ω_robot = (v_right - v_left) / wheelbase
```

**Inverse Kinematics** (robot velocity → wheel speeds):
```
v_left  = v_robot - (ω_robot × wheelbase / 2)
v_right = v_robot + (ω_robot × wheelbase / 2)
```

**Velocity Limits**:
- Max linear velocity: ~0.26 m/s (at 300 RPM)
- Max angular velocity: ~1.29 rad/s

## Architecture

```
Navigation Stack
      ↓ /cmd_vel (Twist)
┌─────────────────────────┐
│  HoverBot Driver Node   │
│  ┌──────────────────┐   │
│  │ Twist → Wheels   │   │  ← Differential drive kinematics
│  └──────────────────┘   │
│  ┌──────────────────┐   │
│  │ Serial Interface │   │  ← Protocol encoding/decoding
│  └──────────────────┘   │
└─────────────────────────┘
      ↓ UART (8-byte packets, 50Hz)
┌─────────────────────────┐
│   Hoverboard Firmware   │
│  (EFeru FOC, USART3)    │
└─────────────────────────┘
      ↑ Telemetry (18-byte packets, 100Hz)
┌─────────────────────────┐
│  HoverBot Driver Node   │
│  ┌──────────────────┐   │
│  │ Wheels → Odom    │   │  ← Dead reckoning
│  └──────────────────┘   │
└─────────────────────────┘
      ↓ /odom, /tf
Navigation Stack
```

## Troubleshooting

### Hoverboard beeping continuously
- **Cause**: Firmware not receiving valid commands (timeout)
- **Fix**: Ensure driver node is running at 50Hz, check serial connection

### No wheel movement
- **Check**: Serial port permissions (`sudo chmod 666 /dev/ttyAMA0`)
- **Check**: Correct UART pins connected (TX→RX, RX→TX, GND→GND)
- **Check**: Firmware has `TANK_STEERING` enabled
- **Check**: Battery voltage sufficient (> 30V)

### Odometry drift
- **Expected**: Dead reckoning accumulates error over time
- **Fix**: Integrate with SLAM (e.g., `slam_toolbox`) for drift correction
- **Tune**: Adjust wheel_diameter and wheelbase parameters for better accuracy

### High checksum/framing errors
- **Check**: Baud rate matches firmware (115200)
- **Check**: UART wiring quality (short cables, proper grounding)
- **Check**: No electrical noise interference

## Integration with Navigation

Once the driver is working, integrate with Nav2:

```bash
# 1. Launch driver
ros2 launch hoverbot_driver hoverbot_driver.launch.py

# 2. Launch robot state publisher (from hoverbot_description)
ros2 launch hoverbot_description robot_state_publisher.launch.py

# 3. Launch SLAM (creates map while driving)
ros2 launch hoverbot_bringup slam.launch.py

# 4. Launch navigation (autonomous navigation)
ros2 launch hoverbot_bringup navigation.launch.py
```

## Testing Checklist

- [ ] Driver node starts without errors
- [ ] `/odom` topic publishing at 50Hz
- [ ] TF transform `odom → base_link` broadcasting
- [ ] Wheels respond to `/cmd_vel` commands
- [ ] Odometry x, y, theta updates when moving
- [ ] Battery voltage and temperature in `/diagnostics`
- [ ] No checksum or framing errors in diagnostics
- [ ] Robot stops when cmd_vel timeout expires
- [ ] Clean shutdown sends zero command

## Performance Notes

- **Control Loop**: 50Hz (required for firmware heartbeat)
- **Odometry Update**: 50Hz (matches control loop)
- **Telemetry Rate**: 100Hz (firmware sends feedback)
- **Diagnostic Publishing**: 5Hz (sufficient for monitoring)

## Safety Features

1. **Command Timeout**: Robot stops if no `/cmd_vel` for 0.5s
2. **Velocity Limiting**: Commands clamped to safe RPM limits
3. **Checksum Validation**: Corrupted packets discarded
4. **Clean Shutdown**: Zero command sent before disconnect

## License

MIT License - See LICENSE file for details

## Credits

- **EFeru**: Original hoverboard-firmware-hack-FOC
- **Ryan**: HoverBot integration and ROS 2 driver
- **Community**: Testing and feedback

## Support

For issues specific to:
- **This driver**: Open issue on HoverBot repository
- **Firmware**: See [EFeru issues](https://github.com/EFeru/hoverboard-firmware-hack-FOC/issues)
- **ROS 2 Nav2**: See [Nav2 documentation](https://navigation.ros.org/)
