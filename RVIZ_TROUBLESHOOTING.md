# RViz2 Troubleshooting Guide - Laser and Map Not Showing

## Quick Diagnosis on Dev Machine

Run these commands on your **Dev machine** to diagnose the issue:

### Step 1: Verify Topics Are Publishing

```bash
# List all available topics
ros2 topic list

# Check if /scan topic exists
ros2 topic list | grep scan

# Check if /map topic exists
ros2 topic list | grep map

# Check scan topic details
ros2 topic info /scan

# Echo a few scan messages to verify data
ros2 topic echo /scan --once
```

**Expected output:**
- `/scan` should be in the topic list
- `/map` should be in the topic list (if SLAM is running)
- Type should be `sensor_msgs/msg/LaserScan` for /scan
- Type should be `nav_msgs/msg/OccupancyGrid` for /map

### Step 2: Check Topic Rates

```bash
# Check scan publish rate (should be ~7-10 Hz)
ros2 topic hz /scan

# Check map publish rate (should be ~0.1 Hz, every 10 seconds)
ros2 topic hz /map
```

### Step 3: Check TF Frames

```bash
# List all TF frames
ros2 run tf2_ros tf2_echo odom laser

# View entire TF tree
ros2 run tf2_tools view_frames
# This creates frames.pdf - open it to see the frame tree
```

**Expected frames:**
- `odom` (root frame from driver)
- `base_link` (robot center)
- `laser` (LiDAR sensor frame)

---

## Common Issues and Fixes

### Issue 1: Topics Not Visible on Dev Machine

**Problem:** `ros2 topic list` on Dev doesn't show `/scan` or `/map`

**Cause:** Network/ROS_DOMAIN_ID mismatch

**Fix:**
```bash
# On Dev machine - check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Should be 0 (or match Pi4)
# If not set:
export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Verify you can ping Pi4
ping <pi4-ip-address>

# Check firewall (if needed)
sudo ufw allow from <pi4-ip-address>
```

### Issue 2: Wrong Fixed Frame in RViz2

**Problem:** RViz shows "Fixed Frame [map] does not exist"

**Cause:** Wrong fixed frame selected

**Fix in RViz2:**
1. In left panel, find **Global Options**
2. Change **Fixed Frame** from `map` to `odom`
3. Alternatively try: `base_link` or `laser`

**Correct frame hierarchy:**
```
odom → base_link → laser
```

### Issue 3: Wrong LaserScan Topic

**Problem:** LaserScan display shows nothing or errors

**Cause:** Wrong topic name

**Fix in RViz2:**
1. Click on **LaserScan** display in left panel
2. Check **Topic** field
3. Should be: `/scan` (NOT `/laser/scan` or `/rplidar/scan`)
4. Click the dropdown to see available topics
5. Select `/scan` from the list

**Alternative:** Check what the actual scan topic is:
```bash
ros2 topic list | grep -i scan
```
Use whatever topic name appears.

### Issue 4: Map Not Displaying

**Problem:** Map display exists but shows nothing

**Causes & Fixes:**

**A) SLAM not running yet:**
```bash
# Check if slam_toolbox is running
ros2 node list | grep slam

# If not in list, SLAM hasn't started
# Wait 17+ seconds from launch, or start manually:
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=false
```

**B) Map not publishing yet:**
```bash
# SLAM publishes map slowly (~0.1 Hz = every 10 seconds)
# Wait at least 10-20 seconds, then check:
ros2 topic hz /map

# If "no messages received", drive robot to create map:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**C) Wrong map topic:**
1. In RViz2, click **Map** display
2. Check **Topic** field
3. Should be: `/map`
4. Click dropdown to see available topics

**D) Wrong durability setting:**
1. In RViz2, click **Map** display
2. Find **Durability Policy**
3. Try changing between: `Transient Local` and `Volatile`
4. SLAM typically uses `Transient Local`

### Issue 5: Frame Transform Errors

**Problem:** "Transform [sender=unknown_publisher] For frame [laser]: Frame [laser] does not exist"

**Cause:** Static transform not published

**Fix:** Check if static transform node is running:
```bash
# List all nodes
ros2 node list

# Should see: /base_to_laser_tf
# If missing, the launch file didn't start it properly

# Manually publish it:
ros2 run tf2_ros static_transform_publisher \
  --x 0.1 --y 0.0 --z 0.1 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id laser
```

---

## Correct RViz2 Configuration

### LaserScan Display Settings:
- **Topic:** `/scan`
- **Size (m):** `0.05` (adjust for visibility)
- **Style:** `Points` or `Flat Squares`
- **Color Transformer:** `Intensity` or `FlatColor`
- **Decay Time:** `0` (no decay)

### Map Display Settings:
- **Topic:** `/map`
- **Color Scheme:** `map`
- **Alpha:** `0.7`
- **Draw Behind:** `false`
- **Update Topic:** (leave empty)
- **Durability Policy:** `Transient Local`

### TF Display Settings:
- Enable **TF** display
- **Show Names:** `true`
- **Show Axes:** `true`
- **Show Arrows:** `true`
- **Marker Scale:** `0.3`

### Global Options:
- **Fixed Frame:** `odom` (start here)
- **Background Color:** `48; 48; 48` (dark gray)

---

## Complete Diagnostic Script

Run this on **Dev machine** to get full diagnostic output:

```bash
#!/bin/bash
echo "=== ROS 2 Environment ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_DISTRO: $ROS_DISTRO"
echo ""

echo "=== Available Topics ==="
ros2 topic list
echo ""

echo "=== /scan Topic Info ==="
ros2 topic info /scan
echo ""

echo "=== /map Topic Info ==="
ros2 topic info /map
echo ""

echo "=== Active Nodes ==="
ros2 node list
echo ""

echo "=== TF Frames ==="
ros2 run tf2_ros tf2_echo odom laser
```

Save as `diagnose_rviz.sh`, make executable: `chmod +x diagnose_rviz.sh`

---

## Step-by-Step RViz2 Setup

### 1. Start RViz2
```bash
rviz2
```

### 2. Set Fixed Frame
- Left panel → **Global Options**
- **Fixed Frame:** Change to `odom`

### 3. Add LaserScan Display
- Click **Add** button (bottom left)
- Select **By topic** tab
- Find `/scan` → **LaserScan**
- Click **OK**

### 4. Configure LaserScan
- Click **LaserScan** in left panel
- **Size:** `0.05`
- **Color Transformer:** `Intensity`
- Should see laser dots immediately if data is flowing

### 5. Add Map Display
- Click **Add** button
- Select **By topic** tab
- Find `/map` → **Map**
- Click **OK**

### 6. Configure Map
- Click **Map** in left panel
- **Durability Policy:** `Transient Local`
- Wait 10-20 seconds for map to appear
- Drive robot if map is empty

### 7. Add TF Display (Optional)
- Click **Add** button
- Select **By display type** tab
- Find **TF**
- Click **OK**
- See frame axes and relationships

### 8. Save Configuration
- **File** → **Save Config As**
- Save to: `~/hoverbot/config/hoverbot_dev.rviz`

Next time: `rviz2 -d ~/hoverbot/config/hoverbot_dev.rviz`

---

## Still Not Working?

### Nuclear Option - Full Reset:

```bash
# On Dev machine:
# 1. Kill RViz2
killall rviz2

# 2. Remove RViz2 config cache
rm -rf ~/.rviz2

# 3. Verify ROS is seeing Pi4
ros2 topic list

# 4. Restart RViz2
rviz2

# 5. Manually add displays from scratch
```

### Check Pi4 is Publishing:

SSH to Pi4 and verify:
```bash
ssh hoverbot

# Check nodes are running
ros2 node list

# Should see:
# /hoverbot_driver_node
# /rplidar_node (or similar)
# /slam_toolbox
# /base_to_laser_tf

# Check scan is publishing
ros2 topic hz /scan
# Should show ~7-10 Hz

# Check scan data
ros2 topic echo /scan --once
```

---

## Quick Reference Card

| Issue | Check | Fix |
|-------|-------|-----|
| No topics visible | `ros2 topic list` empty on Dev | Check `ROS_DOMAIN_ID=0` on both machines |
| LaserScan blank | Wrong topic | Change to `/scan` in RViz |
| Map not showing | SLAM not started | Wait 17+ seconds or drive robot |
| Frame errors | TF not published | Check `ros2 node list` for `/base_to_laser_tf` |
| "Fixed frame does not exist" | Wrong fixed frame | Change to `odom` in Global Options |

---

**Most Common Solution:**
1. Fixed Frame: `odom` (not `map`)
2. LaserScan Topic: `/scan`
3. Map Durability: `Transient Local`
4. Wait 20+ seconds after launch for SLAM to initialize
5. Drive robot to see map build
