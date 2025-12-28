# Nav2 Autonomous Navigation Setup

## Overview
Nav2 (Navigation2) enables HoverBot to autonomously navigate through mapped environments, avoiding obstacles and planning paths to goals.

## Prerequisites
- ✅ SLAM mapping completed (have a saved map)
- ✅ Robot hardware assembled (RPLidar mounted on hoverbot)
- ✅ All ROS 2 components working (driver, lidar, SLAM)

## What Nav2 Provides
- **Path Planning:** Calculate routes from current position to goal
- **Obstacle Avoidance:** Dynamic costmaps using lidar data
- **Localization:** AMCL (Adaptive Monte Carlo Localization) for position tracking
- **Recovery Behaviors:** Get unstuck when path is blocked
- **Goal Execution:** Follow planned paths with differential drive controller

## Configuration

### Robot Parameters (Tuned for HoverBot)
- **Max Linear Velocity:** 0.5 m/s (conservative, 2.59 m/s available)
- **Max Angular Velocity:** 1.0 rad/s (conservative, 12.96 rad/s available)
- **Robot Radius:** 0.25 m (adjust based on actual robot size)
- **Lidar Range:** 12.0 m max (RPLidar A1 spec)

### Files Created
- `config/nav2/nav2_params.yaml` - Nav2 stack configuration
- `launch/nav2_bringup.launch.py` - Launch file for Nav2

## Usage Workflow

### 1. Create a Map (One-Time)

**Start SLAM mapping:**
```bash
# On dev machine
cd ~/hoverbot/scripts
./hoverbot_startup.sh

# Drive around with teleop to build map
# (See SLAM_STARTUP.md)
```

**Save the map:**
```bash
# In tmux window or new terminal
ssh ryan@192.168.86.20
source ~/.bashrc
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room

# This creates:
# - ~/maps/my_room.pgm (map image)
# - ~/maps/my_room.yaml (map metadata)
```

### 2. Launch Robot with Nav2

**Stop SLAM if running:**
```bash
# Kill existing session
tmux kill-session -t hoverbot
```

**Launch robot stack (driver + lidar + TF):**
```bash
cd ~/hoverbot/scripts
# TODO: Create nav2_startup.sh script
# For now, manually launch first 3 components from hoverbot_startup.sh
```

**Launch Nav2 with saved map:**
```bash
# In new terminal
ssh ryan@192.168.86.20
source ~/.bashrc
ros2 launch hoverbot_bringup nav2_bringup.launch.py map:=/home/ryan/maps/my_room.yaml
```

### 3. Set Initial Pose in RViz

**On dev machine, launch RViz:**
```bash
rviz2
```

**Configure RViz:**
1. Fixed Frame: `map`
2. Add displays:
   - Map (`/map`)
   - LaserScan (`/scan`)
   - TF
   - RobotModel (if URDF available)
   - Particle Cloud (`/particle_cloud`) - AMCL localization
   - Global Costmap (`/global_costmap/costmap`)
   - Local Costmap (`/local_costmap/costmap`)
   - Global Plan (`/plan`)
   - Local Plan (`/local_plan`)

**Set initial pose:**
1. Click **"2D Pose Estimate"** button in RViz toolbar
2. Click on map where robot is located
3. Drag to set orientation
4. Particle cloud should converge around robot position

### 4. Send Navigation Goal

**Set goal in RViz:**
1. Click **"2D Nav Goal"** button in RViz toolbar
2. Click on map where you want robot to go
3. Drag to set goal orientation
4. Robot should plan path and start moving!

**Monitor progress:**
- Green line: Global plan (full path)
- Red/yellow: Local plan (current trajectory)
- Colored costmaps: Obstacles and inflation zones

## Testing Without Hardware Assembly

**Current Status:** Hardware not assembled, so full autonomous navigation cannot be tested.

**What CAN be tested:**
- ✅ Nav2 launches without errors
- ✅ Parameters load correctly
- ✅ Costmaps generate from lidar data
- ✅ Path planner can plan paths on saved map

**What CANNOT be tested:**
- ❌ Actual robot movement to goals
- ❌ Dynamic obstacle avoidance while moving
- ❌ Localization accuracy
- ❌ Path following performance

## Command Reference

**Save a map:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/map_name
```

**Launch Nav2:**
```bash
ros2 launch hoverbot_bringup nav2_bringup.launch.py map:=/path/to/map.yaml
```

**Send goal programmatically:**
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'
```

**Check Nav2 status:**
```bash
ros2 node list | grep nav2
ros2 topic list | grep nav2
```

## Tuning Parameters

### Increase Speed (After Testing)
Edit `config/nav2/nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 1.0  # Increase from 0.5 m/s
      max_vel_theta: 2.0  # Increase from 1.0 rad/s
```

### Adjust Robot Size
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.30  # Increase if robot is larger
```

### Tune Goal Tolerance
```yaml
controller_server:
  ros__parameters:
    general_goal_checker:
      xy_goal_tolerance: 0.10  # Tighter goal (default 0.15)
      yaw_goal_tolerance: 0.15  # Tighter orientation (default 0.2)
```

## Troubleshooting

### "No path could be found"
- Check costmaps in RViz - is goal in obstacle?
- Verify map is loaded correctly
- Check robot localization (particle cloud)
- Increase planner tolerance

### Robot doesn't move
- Check `/cmd_vel` topic is being published
- Verify hoverboard driver is receiving commands
- Check for costmap obstacles blocking path
- Verify controller is active: `ros2 node info /controller_server`

### Poor localization (particles scattered)
- Set better initial pose estimate
- Drive around to help AMCL converge
- Increase particle count in AMCL params
- Check lidar is publishing clean scans

### Robot gets stuck
- Check recovery behaviors are enabled
- Verify local costmap is clearing properly
- Tune inflation radius (might be too large)
- Check for sensor issues (lidar blocked)

## Next Steps (When Hardware Ready)

1. **Assemble robot** - Mount RPLidar on hoverbot chassis
2. **Create test map** - Map small area for initial testing
3. **Test localization** - Verify AMCL tracks position accurately
4. **Test simple goals** - Short distances, verify path following
5. **Tune parameters** - Adjust speeds, tolerances based on performance
6. **Test obstacle avoidance** - Place obstacles, verify dynamic replanning
7. **Create production maps** - Map full deployment environment

## Status
✅ Nav2 configured and ready - December 27, 2025
⏳ Awaiting hardware assembly for testing
✅ Parameters tuned for differential drive hoverbot
✅ Conservative velocity limits for safe initial testing
