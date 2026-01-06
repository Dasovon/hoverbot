# BNO055 IMU Driver for ROS2

A custom ROS2 driver for the Bosch BNO055 9-axis Absolute Orientation Sensor, using the Adafruit CircuitPython library.

## Overview

This driver provides a ROS2 interface to the BNO055 IMU sensor, publishing standard sensor messages for:

- **Orientation** (fused from accelerometer, gyroscope, and magnetometer)
- **Angular velocity** (from gyroscope)
- **Linear acceleration** (from accelerometer)
- **Magnetic field** (from magnetometer)
- **Temperature** (from internal sensor)

The BNO055 features on-chip sensor fusion, providing accurate absolute orientation as a quaternion without the need for external sensor fusion algorithms.

## Why This Driver?

The existing `flynneva/bno055` ROS2 driver was found to be incompatible with our hardware setup - it would hang on startup and fail to publish any topics. After verifying the hardware was functioning correctly via direct I2C testing, we created this lightweight custom driver using the proven Adafruit CircuitPython library.

This driver is simpler, more reliable, and easier to debug than the existing alternatives.

## Hardware Setup

### Wiring

Connect the BNO055 to your Raspberry Pi I2C bus:

| BNO055 Pin | Raspberry Pi Pin | Description         |
| ---------- | ---------------- | ------------------- |
| VIN        | Pin 1 (3.3V)     | Power supply (3.3V) |
| GND        | Pin 6 (Ground)   | Ground              |
| SDA        | Pin 3 (GPIO 2)   | I2C Data            |
| SCL        | Pin 5 (GPIO 3)   | I2C Clock           |

**Note:** The BNO055 operates at **3.3V**. Do not connect to 5V.

### I2C Address

The driver expects the BNO055 at I2C address **0x28** (default when ADR pin is low).

To verify your sensor is detected:

```bash
sudo i2cdetect -y 1
```

You should see `28` in the output grid.

### I2C Configuration

Ensure I2C is enabled on your Raspberry Pi:

```bash
# Check if I2C is enabled
ls /dev/i2c-1

# If not found, enable I2C using raspi-config
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable
```

## Installation

### Dependencies

Install the Adafruit CircuitPython library:

```bash
pip3 install adafruit-circuitpython-bno055
```

For system-wide installation (recommended for ROS2):

```bash
sudo pip3 install adafruit-circuitpython-bno055
```

### Building the Package

```bash
cd ~/hoverbot/ros2_ws
colcon build --packages-select bno055_driver
source install/setup.bash
```

## Usage

### Launch the Driver

```bash
ros2 launch bno055_driver bno055.launch.py
```

Or run the node directly:

```bash
ros2 run bno055_driver bno055_node
```

### Verify Topics

Check that topics are publishing:

```bash
# List topics
ros2 topic list | grep imu

# Check publishing rate
ros2 topic hz /imu/data
ros2 topic hz /imu/mag

# View data
ros2 topic echo /imu/data
```

## Published Topics

| Topic       | Message Type                | Rate  | Description                                                     |
| ----------- | --------------------------- | ----- | --------------------------------------------------------------- |
| `/imu/data` | `sensor_msgs/Imu`           | 50 Hz | Orientation (quaternion), angular velocity, linear acceleration |
| `/imu/mag`  | `sensor_msgs/MagneticField` | 50 Hz | Magnetic field vector in Tesla                                  |
| `/imu/temp` | `sensor_msgs/Temperature`   | 1 Hz  | Internal sensor temperature in Celsius                          |

### Message Details

**`/imu/data` (sensor_msgs/Imu):**

- `orientation`: Absolute orientation as quaternion (fused from all sensors)
- `angular_velocity`: Rotation rates in rad/s (x, y, z)
- `linear_acceleration`: Linear acceleration in m/s² (x, y, z)
- All fields include covariance matrices

**`/imu/mag` (sensor_msgs/MagneticField):**

- `magnetic_field`: Magnetic field strength in Tesla (x, y, z)
- Includes covariance matrix

**`/imu/temp` (sensor_msgs/Temperature):**

- `temperature`: Sensor temperature in Celsius
- Useful for monitoring thermal stability

## Parameters

Configure the driver via ROS2 parameters:

| Parameter      | Type   | Default      | Description                                    |
| -------------- | ------ | ------------ | ---------------------------------------------- |
| `frame_id`     | string | `'imu_link'` | TF frame for published messages                |
| `publish_rate` | double | `50.0`       | Publishing rate in Hz (recommended: 10-100 Hz) |

### Setting Parameters

Via launch file:

```python
Node(
    package='bno055_driver',
    executable='bno055_node',
    parameters=[{
        'frame_id': 'base_imu',
        'publish_rate': 100.0
    }]
)
```

Via command line:

```bash
ros2 run bno055_driver bno055_node --ros-args \
    -p frame_id:=base_imu \
    -p publish_rate:=100.0
```

## Sensor Calibration

The BNO055 requires calibration for optimal performance. Calibration status is stored on-chip and persists between reboots (once fully calibrated).

### Calibration Procedure

1. **Gyroscope:** Place the sensor on a stable surface and leave motionless for a few seconds.

2. **Accelerometer:** Slowly move the sensor through various orientations:

   - Hold flat (face up)
   - Hold flat (face down)
   - Stand on each edge
   - Stand on each corner

3. **Magnetometer:** Move the sensor in a figure-8 pattern through the air. Repeat several times in different orientations.

### Checking Calibration Status

The BNO055 stores calibration status internally. You can read it via Python:

```python
import board
import adafruit_bno055

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Returns tuple: (system, gyro, accel, mag) from 0 (uncalibrated) to 3 (fully calibrated)
print(sensor.calibration_status)
```

Fully calibrated sensors will show `(3, 3, 3, 3)`.

### Calibration Tips

- Calibrate in the environment where the robot will operate
- Avoid magnetic interference (keep away from metal, magnets, power supplies)
- For best results, calibrate outdoors or in a magnetically clean area
- The sensor automatically saves calibration data - no need to manually persist it

## Integration with Robot

### TF Frames

The IMU publishes data in the frame specified by the `frame_id` parameter (default: `imu_link`). You should define this frame in your robot's URDF:

```xml
<link name="imu_link"/>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>  <!-- Adjust to actual mounting position -->
</joint>
```

### Sensor Fusion with robot_localization

The BNO055's orientation output can be fused with other sensors using `robot_localization`:

```yaml
ekf_filter_node:
  ros__parameters:
    imu0: /imu/data
    imu0_config: [
        false,
        false,
        false, # x, y, z position
        true,
        true,
        true, # roll, pitch, yaw orientation
        false,
        false,
        false, # x, y, z velocity
        true,
        true,
        true, # roll, pitch, yaw velocity
        true,
        true,
        true,
      ] # x, y, z acceleration
```

## Troubleshooting

### Sensor Not Detected

**Problem:** `i2cdetect` doesn't show device at 0x28

**Solutions:**

- Check wiring connections (SDA, SCL, VIN, GND)
- Verify I2C is enabled: `ls /dev/i2c-1`
- Check for I2C address conflicts
- Ensure ADR pin is low (address 0x28) or high (address 0x29)
- Try different I2C pull-up resistors (BNO055 breakout boards usually include them)

### "Failed to initialize BNO055" Error

**Problem:** Node fails with initialization error

**Solutions:**

- Verify Adafruit library is installed: `pip3 list | grep adafruit-circuitpython-bno055`
- Check I2C permissions: user must be in `i2c` and `gpio` groups
  ```bash
  sudo usermod -a -G i2c,gpio $USER
  # Log out and back in for changes to take effect
  ```
- Ensure no other process is using I2C bus
- Power cycle the sensor (disconnect/reconnect power)

### Topics Not Publishing

**Problem:** Node starts but no topics appear

**Solutions:**

- Check node is running: `ros2 node list`
- Check for error messages in node output
- Verify sensor is detected: `i2cdetect -y 1`
- Try lowering publish rate: `-p publish_rate:=10.0`
- Check ROS_DOMAIN_ID matches your network

### Invalid Data (None values)

**Problem:** Sensor returns `None` for orientation or other fields

**Solutions:**

- **Sensor not calibrated:** Perform calibration procedure (especially magnetometer)
- Wait 1-2 seconds after startup for sensor initialization
- Check for electromagnetic interference
- Verify sensor is not in CONFIG mode (should auto-enter NDOF mode)

### Orientation Drifts or Jumps

**Problem:** Orientation quaternion is unstable

**Solutions:**

- **Calibrate magnetometer:** This is the most common cause
- Keep sensor away from magnetic interference (motors, power wires, metal)
- Ensure sensor is mounted rigidly to the robot (vibration affects accuracy)
- Check for loose wiring connections
- Consider using only gyro/accel if operating in magnetically noisy environment

### High CPU Usage

**Problem:** BNO055 node uses excessive CPU

**Solutions:**

- Lower publish rate: `-p publish_rate:=10.0` (default 50 Hz may be overkill)
- The Adafruit library polls over I2C - this is normal
- Consider publishing temperature at lower rate (already done - 1Hz)

## Technical Details

### Sensor Specifications

- **Accelerometer:** ±2g/±4g/±8g/±16g ranges
- **Gyroscope:** ±125°/s to ±2000°/s ranges
- **Magnetometer:** ±1300 µT (x, y), ±2500 µT (z)
- **Fusion Output Rate:** Up to 100 Hz
- **Interface:** I2C (up to 400 kHz Fast Mode)
- **Operating Voltage:** 2.4V to 3.6V (3.3V nominal)
- **Operating Temperature:** -40°C to +85°C

### Coordinate Frames

The BNO055 uses a right-handed coordinate system:

- **X-axis:** Forward (in the direction of the dot on the chip)
- **Y-axis:** Right (when viewing chip from above with dot forward)
- **Z-axis:** Down (into the chip)

Ensure your `frame_id` TF transform accounts for the sensor's physical mounting orientation.

### Sensor Fusion Mode

This driver uses the BNO055's default **NDOF (Nine Degrees of Freedom)** fusion mode:

- Absolute orientation from all 9 axes
- Magnetometer provides heading reference (no yaw drift)
- Best accuracy for most applications
- Requires magnetometer calibration

## Performance Notes

- **Latency:** I2C polling adds ~2-5ms latency per read cycle
- **Jitter:** Timing is software-based, expect ±1-2ms jitter
- **CPU Load:** Minimal (~1-2% on Raspberry Pi 4)
- **Power:** BNO055 draws ~12mA typical

For time-critical applications requiring hard real-time guarantees, consider a CAN-based IMU instead of I2C.

## Package Files

```
bno055_driver/
├── bno055_driver/
│   ├── __init__.py
│   └── bno055_node.py          # Main ROS2 node
├── launch/
│   └── bno055.launch.py        # Launch file
├── test/                       # Package tests
├── package.xml                 # Package metadata
├── setup.py                    # Python package setup
└── README.md                   # This file
```

## Resources

- **BNO055 Datasheet:** https://www.bosch-sensortec.com/products/smart-sensors/bno055/
- **Adafruit CircuitPython Library:** https://github.com/adafruit/Adafruit_CircuitPython_BNO055
- **ROS2 sensor_msgs:** https://docs.ros.org/en/humble/p/sensor_msgs/
- **I2C on Raspberry Pi:** https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c

## License

This package is provided as-is for the HoverBot project. The Adafruit CircuitPython BNO055 library is MIT licensed.

## Author

Created for the HoverBot project (2026-01-05) as a replacement for the incompatible `flynneva/bno055` driver.
