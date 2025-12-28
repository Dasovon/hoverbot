# Parameter Tuning Guide

## Overview
This guide covers optimizing parameters for better SLAM mapping, odometry accuracy, and sensor performance.

## SLAM Toolbox Parameters

**File:** `ros2_ws/src/hoverbot_bringup/config/slam_toolbox_params.yaml`

### Map Quality Settings

**Resolution (meters per pixel):**
```yaml
resolution: 0.05  # Current: 5cm per pixel
```
- **Lower (0.02-0.03):** Higher detail, larger file size, slower processing
- **Higher (0.10-0.15):** Faster processing, less detail
- **Recommended:** 0.05 for indoor mapping (good balance)

**Minimum Travel Distance:**
```yaml
minimum_travel_distance: 0.2  # Current: 20cm
minimum_travel_heading: 0.2   # Current: ~11.5 degrees
```
- **Lower values:** More frequent updates, smoother maps, more CPU
- **Higher values:** Less frequent updates, faster processing
- **Recommended:** 0.2m for typical indoor navigation

**Loop Closure Settings:**
```yaml
loop_search_maximum_distance: 3.0  # Current: 3m search radius
do_loop_closing: true              # Enable/disable
loop_match_minimum_chain_size: 10  # Minimum matches needed
```
- **Larger search distance:** Better loop closure, slower
- **Smaller distance:** Faster, might miss loops
- **For small rooms:** Reduce to 2.0m
- **For large spaces:** Increase to 5.0m

**Scan Matcher Settings:**
```yaml
max_laser_range: 12.0  # RPLidar A1 max range
minimum_time_interval: 0.5  # Minimum time between scans
```
- **max_laser_range:** Match your lidar (12m for RPLidar A1)
- **minimum_time_interval:** Lower for faster updates (0.2-0.3s)

### Memory and Performance
```yaml
max_scan_buffer_size: 10  # Number of scans buffered
```
- **Higher:** Better for fast movement, more memory
- **Lower:** Less memory, might drop scans
- **Recommended:** 10-20 for Pi 4

## Driver Odometry Parameters

**File:** `ros2_ws/src/hoverbot_driver/hoverbot_driver/driver_node.py`

### Wheel Configuration (CRITICAL for accuracy)
```python
# Current values
WHEEL_DIAMETER = 0.165  # meters (16.5cm)
WHEELBASE = 0.40        # meters (40cm)
```

**How to measure:**
1. **Wheel Diameter:** Measure tire outer diameter
2. **Wheelbase:** Distance between wheel centers
3. **Test:** Drive 1 meter, check odometry vs actual
4. **Adjust:** If odometry says 1.1m but actual is 1m, reduce diameter by 10%

**Calibration procedure:**
```bash
# 1. Mark starting position on floor
# 2. Reset odometry (restart driver)
# 3. Drive forward exactly 2 meters (measured with tape)
# 4. Check odometry: ros2 topic echo /odom --once
# 5. Calculate error: error = (odom_distance - actual_distance) / actual_distance
# 6. Adjust: new_diameter = old_diameter * (1 - error)
```

### Odometry Covariance
```python
# Position uncertainty (x, y) in meters²
self.odom_msg.pose.covariance[0] = 0.01   # x variance
self.odom_msg.pose.covariance[7] = 0.01   # y variance
self.odom_msg.pose.covariance[35] = 0.05  # theta variance (rad²)

# Velocity uncertainty
self.odom_msg.twist.covariance[0] = 0.01   # vx variance
self.odom_msg.twist.covariance[35] = 0.05  # vtheta variance
```

**Higher covariance (less confident):**
- Slippery floors → increase to 0.05-0.1
- Soft/deformable wheels → increase
- Poor surface contact

**Lower covariance (more confident):**
- Hard floors, good traction → decrease to 0.005
- Rigid wheels, good contact

### Publish Rate
```python
ODOM_PUBLISH_RATE = 50  # Hz
```
- **Lower (20-30 Hz):** Less CPU, still good
- **Higher (50-100 Hz):** Smoother, more CPU
- **Recommended:** 50 Hz for Pi 4

## RPLidar Filtering

**File:** `ros2_ws/src/hoverbot_bringup/config/slam_toolbox_params.yaml`

### Range Filtering
```yaml
# In SLAM config
max_laser_range: 12.0  # Maximum usable range (meters)
```

**RPLidar A1 specs:**
- Nominal range: 12m
- Effective range: 0.2m - 8m (best quality)

**Recommendations:**
- **Indoor mapping:** Set to 8.0m (better accuracy)
- **Large spaces:** Keep at 12.0m
- **Noisy environment:** Reduce to 6.0m

### Angle Filtering (if needed)

Currently using full 360° scan. To filter (e.g., ignore rear):

**Create custom launch with scan filtering:**
```python
# In launch file, add laser_filters
laser_filter_node = Node(
    package='laser_filters',
    executable='scan_to_scan_filter_chain',
    parameters=[{
        'scan_filter_chain': [
            {
                'name': 'angle',
                'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                'params': {
                    'lower_angle': -2.356,  # -135 degrees
                    'upper_angle': 2.356    # +135 degrees
                }
            }
        ]
    }]
)
```

**When to filter angles:**
- Robot has blind spots or obstructions
- Want to ignore certain directions
- Reduce computation load

## Nav2 Controller Parameters

**File:** `ros2_ws/src/hoverbot_bringup/config/nav2/nav2_params.yaml`

### Velocity Limits (After Testing!)
```yaml
# Current conservative limits
max_vel_x: 0.5        # Linear velocity (m/s)
max_vel_theta: 1.0    # Angular velocity (rad/s)

# Hardware maximum
# max_vel_x: 2.59     # Available from hoverboard
# max_vel_theta: 12.96  # Available
```

**Tuning strategy:**
1. Start conservative (current settings)
2. Test in open area
3. Gradually increase by 0.1 m/s increments
4. Find maximum comfortable speed
5. Set limit to 70% of maximum for safety margin

**Recommended progressive tuning:**
```yaml
# Stage 1: Initial testing (current)
max_vel_x: 0.5
max_vel_theta: 1.0

# Stage 2: After basic validation
max_vel_x: 1.0
max_vel_theta: 2.0

# Stage 3: Performance tuning
max_vel_x: 1.5
max_vel_theta: 3.0

# Final: Production (after extensive testing)
max_vel_x: 2.0  # ~75% of maximum
max_vel_theta: 4.0
```

### Acceleration Limits
```yaml
acc_lim_x: 0.5     # Linear acceleration (m/s²)
acc_lim_theta: 1.0  # Angular acceleration (rad/s²)
```

**Tuning:**
- **Too low:** Sluggish response
- **Too high:** Jerky motion, wheel slip
- **Test:** Command sudden velocity change, observe behavior

### Goal Tolerances
```yaml
xy_goal_tolerance: 0.15    # Position tolerance (meters)
yaw_goal_tolerance: 0.2    # Orientation tolerance (radians ~11°)
```

**Adjust based on needs:**
- **Tighter (0.05m, 0.1rad):** Precise positioning, slower
- **Looser (0.25m, 0.3rad):** Faster arrival, less precise
- **For docking:** Use tight tolerance
- **For waypoint following:** Use loose tolerance

### Costmap Parameters
```yaml
# Robot size
robot_radius: 0.25  # Meters - MEASURE YOUR ROBOT!

# Inflation
inflation_radius: 0.55  # How far to inflate obstacles
cost_scaling_factor: 3.0  # How sharply cost increases
```

**Robot radius:**
1. Measure actual robot (wheel to wheel + margin)
2. Add safety margin (5-10cm)
3. Test in narrow spaces
4. Adjust if robot gets too close to walls

**Inflation radius:**
- **Larger:** Safer, avoids obstacles more
- **Smaller:** Can navigate tighter spaces
- **Recommended:** 2x robot radius minimum

## Testing Procedures

### 1. Odometry Calibration Test
```bash
# Mark floor at robot start position
# Drive straight 2 meters
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}' --once

# Wait 10 seconds, then stop
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}}' --once

# Check odometry
ros2 topic echo /odom --once | grep position

# Measure actual distance traveled
# Calculate error and adjust wheel diameter
```

### 2. SLAM Map Quality Test
```bash
# Drive around room slowly
# Save map: ros2 run nav2_map_server map_saver_cli -f test1

# Change SLAM parameter
# Restart SLAM, drive same path
# Save map: ros2 run nav2_map_server map_saver_cli -f test2

# Compare maps visually in image viewer
```

### 3. Nav2 Performance Test
```bash
# Set navigation goal 5m away
# Observe:
# - Does robot reach goal?
# - Path smoothness
# - Obstacle avoidance distance
# - Goal arrival precision

# Adjust parameters based on behavior
```

## Quick Reference: Common Issues

| Problem | Parameter | Adjustment |
|---------|-----------|------------|
| Map too detailed/slow | resolution | Increase to 0.10 |
| Map not detailed enough | resolution | Decrease to 0.03 |
| Odometry drifts forward | WHEEL_DIAMETER | Decrease by 1-2% |
| Odometry drifts backward | WHEEL_DIAMETER | Increase by 1-2% |
| Robot too cautious | inflation_radius | Decrease by 0.1m |
| Robot hits obstacles | robot_radius | Increase by 0.05m |
| Slow navigation | max_vel_x | Increase by 0.2 m/s |
| Jerky motion | acc_lim_x | Decrease by 0.1 |
| Missing loop closures | loop_search_maximum_distance | Increase to 5.0m |
| Too much CPU usage | minimum_time_interval | Increase to 1.0s |

## Parameter Change Workflow

1. **Document current values** (git commit before changes)
2. **Change ONE parameter at a time**
3. **Test thoroughly** (drive same path multiple times)
4. **Compare results** (save maps/logs)
5. **Keep or revert** based on results
6. **Document findings** in commit message

## Recommended Starting Optimizations

**For typical indoor robot (10-50 square meters):**
```yaml
# SLAM - Faster, slightly less detailed
resolution: 0.08
minimum_travel_distance: 0.15
loop_search_maximum_distance: 4.0

# Nav2 - Moderate speed
max_vel_x: 0.8
max_vel_theta: 1.5
robot_radius: 0.28  # Measure your robot!
```

**For large warehouse (>100 square meters):**
```yaml
# SLAM - Larger search area
loop_search_maximum_distance: 8.0
max_laser_range: 12.0

# Nav2 - Higher speed
max_vel_x: 1.5
max_vel_theta: 3.0
```

## Monitoring Tools
```bash
# Watch odometry in real-time
ros2 topic echo /odom

# Monitor TF accuracy
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link

# Check SLAM processing time
ros2 topic hz /map
ros2 topic bw /scan  # Bandwidth usage

# Nav2 status
ros2 node info /controller_server
ros2 topic echo /diagnostics
```

## Next Steps After Tuning

1. Document optimal parameters found
2. Create "profiles" for different environments
3. Save tuned configs with descriptive names
4. Test endurance (30+ minutes continuous operation)
5. Measure actual vs expected performance

---

**Remember:** Always test parameter changes in safe, controlled environment before deployment!
