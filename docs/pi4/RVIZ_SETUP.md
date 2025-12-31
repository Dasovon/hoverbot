# RViz Visualization Setup

## Overview
RViz allows real-time visualization of the robot from the dev machine, displaying lidar scans, SLAM maps, transforms, and odometry.

## Prerequisites
- HoverBot tmux session running on Pi (see QUICK_START.md)
- Dev machine and Pi on same network (192.168.86.x)
- ROS 2 Humble installed on dev machine

## Installation (Dev Machine)
```bash
sudo apt update
sudo apt install ros-humble-rviz2
```

## Configuration (One-Time Setup)

Add ROS setup to `.bashrc` on dev machine:
```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verify Network Communication
```bash
# On dev machine, check you can see Pi's topics
ros2 topic list

# Should show:
# /cmd_vel, /odom, /scan, /map, /tf, /tf_static, etc.
```

## Launch RViz
```bash
# From any directory on dev machine
rviz2
```

## Configure Displays

### 1. Set Fixed Frame
- In left panel: **Global Options** → **Fixed Frame**
- Set to: `odom` (or `base_link`)

### 2. Add TF (Transform Frames)
- Click **"Add"** button (bottom left)
- Select **"By display type"** tab
- Choose **"TF"**
- Click **"OK"**
- You'll see red/green/blue coordinate frame axes

### 3. Add LaserScan
- Click **"Add"**
- Select **"By topic"** tab
- Expand **`/scan`**
- Select **"LaserScan"**
- Click **"OK"**
- Red/colored dots show lidar points in real-time

### 4. Add Map
- Click **"Add"**
- **"By topic"** tab
- Expand **`/map`**
- Select **"Map"**
- Click **"OK"**
- Wait 10-15 seconds for map to publish (updates every 10 seconds)

### 5. Add Odometry (Optional)
- Click **"Add"**
- **"By topic"** tab
- Expand **`/odom`**
- Select **"Odometry"**
- Click **"OK"**
- Shows robot position as an arrow

## Expected Behavior

### Bench Test (Hardware Not Assembled)
- ✅ **LaserScan:** Updates in real-time, shows environment around lidar
- ✅ **Map:** Displays but doesn't update (odometry static while lidar scans)
- ✅ **TF Frames:** Show coordinate relationships
- ⚠️ **No meaningful mapping:** Robot needs to physically move for SLAM to work

### With Assembled Robot
- ✅ **LaserScan:** Live obstacle detection
- ✅ **Map:** Updates as robot drives around
- ✅ **TF Frames:** Move with robot position
- ✅ **Odometry:** Shows robot path and orientation

## Testing Real-Time Updates

**Move RPLidar by hand:**
- LaserScan dots should update instantly
- Shows RViz receiving live data

**Drive robot with teleop (when assembled):**
- Map should grow as you explore
- Odometry arrow moves
- TF frames update

## Troubleshooting

### "No map received" Warning
- **Cause:** Maps publish every 10 seconds
- **Solution:** Wait 15-20 seconds after opening RViz

### Can't See Topics from Pi
- **Check ROS_DOMAIN_ID:**
```bash
  echo $ROS_DOMAIN_ID  # Should show: 0
```
- **Verify network:** Dev machine and Pi on same subnet
- **Restart RViz:** Close and reopen after setting domain

### LaserScan Not Showing
- **Check topic:** `ros2 topic hz /scan` (should be ~7 Hz)
- **Check RPLidar:** Look at RPLidar pane in tmux
- **Restart components:** Kill and rerun tmux startup script

## Save RViz Configuration

Once you have displays configured:

1. **File** → **Save Config As**
2. Save to: `~/hoverbot/config/hoverbot.rviz`
3. Next time: **File** → **Open Config** → select saved file

## Quick Reference

**Start visualization:**
```bash
# Terminal 1: Start robot
cd ~/hoverbot/scripts
./hoverbot_startup.sh

# Terminal 2: Start RViz (after robot is running)
rviz2
```

**Verify data flow:**
```bash
ros2 topic hz /scan   # ~7 Hz
ros2 topic hz /odom   # ~50 Hz  
ros2 topic hz /map    # ~0.1 Hz
```

## Status
✅ Tested and working - December 27, 2024
✅ Real-time lidar visualization confirmed
✅ Map display functional
✅ Ready for physical robot assembly
