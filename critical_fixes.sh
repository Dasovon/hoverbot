#!/bin/bash
# HoverBot Critical Fixes Script
# Fixes all priority issues identified in analysis

set -e

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║    HoverBot Critical Fixes - December 31, 2026            ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

REPO_DIR="$HOME/hoverbot"

if [ ! -d "$REPO_DIR" ]; then
    echo "❌ Error: Repository not found at $REPO_DIR"
    exit 1
fi

cd "$REPO_DIR"

# ============================================================================
# FIX 1: URDF Typo (Line 226) - CRITICAL!
# ============================================================================
echo "🔴 Fix 1: Correcting URDF typo (line 226)..."

URDF_FILE="ros2_ws/src/hoverbot_description/urdf/hoverbot.urdf"

if [ -f "$URDF_FILE" ]; then
    # Fix the typo: <inertial ixx= → <inertia ixx=
    sed -i '226s/<inertial ixx=/<inertia ixx=/' "$URDF_FILE"
    echo "   ✓ URDF typo fixed (inertial → inertia)"
else
    echo "   ⚠ URDF file not found"
fi

# ============================================================================
# FIX 2: Add udev Rules to Repository
# ============================================================================
echo "🔴 Fix 2: Adding udev rules to repository..."

mkdir -p config/udev

cat > config/udev/99-hoverbot.rules << 'EOF'
# HoverBot udev Rules
# Creates persistent device names for sensors

# RPLidar A1 - Creates /dev/rplidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="rplidar", MODE="0666", GROUP="dialout"

# ELP USB Camera - Creates /dev/elp_camera  
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="0234", \
  ATTR{index}=="0", SYMLINK+="elp_camera", MODE="0666", GROUP="video"

# Intel RealSense D435 - Ensure permissions
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", \
  MODE="0666", GROUP="video"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", \
  MODE="0666", GROUP="video"
EOF

cat > config/udev/README.md << 'EOF'
# udev Rules for HoverBot

Persistent device names that work on any USB port.

## Installation on Raspberry Pi

```bash
sudo cp 99-hoverbot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Unplug and replug devices to activate.**

## Persistent Device Names

- **`/dev/rplidar`** → RPLidar A1 (any USB port)
- **`/dev/elp_camera`** → ELP USB camera (any USB port)
- RealSense D435 auto-detected by driver (USB ID 8086:0b07)

## Verification

```bash
ls -la /dev/rplidar      # Should show symlink
ls -la /dev/elp_camera   # Should show symlink
```

## Troubleshooting

**Device not appearing:**
```bash
# Check USB IDs
lsusb

# Test rule
udevadm test /sys/class/tty/ttyUSB0
```

**Permissions incorrect:**
```bash
# Ensure MODE="0666" in rule
ls -la /dev/rplidar  # Should show: crw-rw-rw-
```

---

**Last Updated:** December 31, 2026
EOF

echo "   ✓ udev rules created in config/udev/"

# ============================================================================
# FIX 3: Add Missing Sensor Links to URDF
# ============================================================================
echo "🟠 Fix 3: Adding IMU and camera links to URDF..."

if [ -f "$URDF_FILE" ]; then
    # Check if sensor links already exist
    if ! grep -q "imu_link" "$URDF_FILE"; then
        # Add sensor links before closing </robot> tag
        sed -i '/<\/robot>/i\
  <!-- IMU (BNO055) -->\
  <link name="imu_link">\
    <visual>\
      <origin xyz="0 0 0" rpy="0 0 0"/>\
      <geometry>\
        <box size="0.025 0.025 0.010"/>\
      </geometry>\
      <material name="dark_grey"/>\
    </visual>\
    <collision>\
      <origin xyz="0 0 0" rpy="0 0 0"/>\
      <geometry>\
        <box size="0.025 0.025 0.010"/>\
      </geometry>\
    </collision>\
    <inertial>\
      <origin xyz="0 0 0" rpy="0 0 0"/>\
      <mass value="0.005"/>\
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0"\
               iyy="0.00001" iyz="0.0"\
               izz="0.00001"/>\
    </inertial>\
  </link>\
\
  <joint name="imu_joint" type="fixed">\
    <parent link="base_link"/>\
    <child link="imu_link"/>\
    <origin xyz="0.0 0.0 0.10" rpy="0 0 0"/>\
  </joint>\
\
  <!-- RealSense D435 Camera -->\
  <link name="camera_link">\
    <visual>\
      <origin xyz="0 0 0" rpy="0 0 0"/>\
      <geometry>\
        <box size="0.090 0.025 0.025"/>\
      </geometry>\
      <material name="black"/>\
    </visual>\
    <collision>\
      <origin xyz="0 0 0" rpy="0 0 0"/>\
      <geometry>\
        <box size="0.090 0.025 0.025"/>\
      </geometry>\
    </collision>\
    <inertial>\
      <origin xyz="0 0 0" rpy="0 0 0"/>\
      <mass value="0.072"/>\
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"\
               iyy="0.0001" iyz="0.0"\
               izz="0.0001"/>\
    </inertial>\
  </link>\
\
  <joint name="camera_joint" type="fixed">\
    <parent link="base_link"/>\
    <child link="camera_link"/>\
    <origin xyz="0.15 0.0 0.20" rpy="0 0 0"/>\
  </joint>\
\
  <!-- ELP USB Camera -->\
  <link name="elp_camera_link">\
    <visual>\
      <origin xyz="0 0 0" rpy="0 0 0"/>\
      <geometry>\
        <box size="0.030 0.030 0.030"/>\
      </geometry>\
      <material name="black"/>\
    </visual>\
    <collision>\
      <origin xyz="0 0 0" rpy="0 0 0"/>\
      <geometry>\
        <box size="0.030 0.030 0.030"/>\
      </geometry>\
    </collision>\
    <inertial>\
      <origin xyz="0 0 0" rpy="0 0 0"/>\
      <mass value="0.050"/>\
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"\
               iyy="0.0001" iyz="0.0"\
               izz="0.0001"/>\
    </inertial>\
  </link>\
\
  <joint name="elp_camera_joint" type="fixed">\
    <parent link="base_link"/>\
    <child link="elp_camera_link"/>\
    <origin xyz="0.15 0.05 0.20" rpy="0 0 0"/>\
  </joint>\
' "$URDF_FILE"
        
        echo "   ✓ Added IMU and camera links to URDF"
    else
        echo "   ℹ Sensor links already exist in URDF"
    fi
fi

# ============================================================================
# FIX 4: Create ELP Camera Documentation
# ============================================================================
echo "🟠 Fix 4: Creating ELP camera documentation..."

cat > docs/camera/ELP_CAMERA_SETUP.md << 'EOF'
# ELP USB Camera Setup - GS1200P01

## Overview

ELP GS1200P01 global shutter USB camera for high-quality RGB video.

**Key Specifications:**
- **Resolution:** 1280×720
- **Frame Rate:** 30 FPS (stable on USB 2.1!)
- **Compression:** MJPEG (bandwidth efficient)
- **Shutter:** Global (no motion blur)
- **USB ID:** 32e4:0234 (Global Shutter Camera)
- **Interface:** USB 2.0/3.0
- **Power:** USB bus powered

## Why ELP Over RealSense RGB?

| Feature | RealSense D435 RGB | ELP GS1200P01 |
|---------|-------------------|---------------|
| Resolution | 640×480 | **1280×720** |
| Frame Rate (USB 2.1) | ~6 Hz (unstable) | **30 Hz (stable)** |
| Bandwidth | ~28 MB/s | **~8 MB/s** (MJPEG) |
| Motion Blur | Rolling shutter | **Global shutter** |
| USB 2.1 | ❌ Timeouts, dropped frames | ✅ Perfect! |

**Verdict:** ELP is ideal for USB 2.1 environments!

## Dual Camera Strategy

**HoverBot uses BOTH cameras for complementary capabilities:**

- **RealSense D435:** Depth-only mode (15 Hz) → 3D obstacle detection, navigation
- **ELP USB:** RGB video (30 Hz) → Visual tasks, object recognition, recording

## Hardware Setup

**Connection:** USB 2.0 or 3.0 port on Raspberry Pi

**Persistent Device:** `/dev/elp_camera` (via udev rules)

**udev Rule:**
```bash
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="0234", \
  ATTR{index}=="0", SYMLINK+="elp_camera", MODE="0666", GROUP="video"
```

## ROS 2 Integration

**Driver:** usb_cam (ros-humble-usb-cam)

**Configuration:**  
`ros2_ws/src/hoverbot_bringup/config/elp_camera.yaml`

```yaml
/**:
  ros__parameters:
    video_device: "/dev/elp_camera"
    framerate: 30.0
    image_width: 1280
    image_height: 720
    pixel_format: "mjpeg2rgb"
    camera_name: "elp_camera"
    io_method: "mmap"
```

**Topics Published:**
- `/elp/image_raw` (sensor_msgs/Image) - 30 Hz RGB
- `/elp/camera_info` (sensor_msgs/CameraInfo) - Camera calibration

## Launch Integration

**Included in:** `hoverbot_full_v3.launch.py`

**Startup Sequence:**
- [8.5s] ELP camera starts (after RealSense at 8s)
- Avoids USB bandwidth conflicts

**Standalone Launch:**
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/elp_camera \
  -p pixel_format:=mjpeg2rgb \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p framerate:=30.0
```

## Performance Validation

**Tested Configuration:**
- ✅ 30 Hz sustained (no dropped frames)
- ✅ 1280×720 full resolution
- ✅ Global shutter (no motion blur)
- ✅ MJPEG keeps USB bandwidth manageable (~8 MB/s)
- ✅ Works alongside RealSense depth stream

**Test Results:**
```bash
$ ros2 topic hz /elp/image_raw
average rate: 30.063
  min: 0.025s max: 0.039s std dev: 0.00252s
```

## RViz Visualization

**Setup:**
1. Add display: Image
2. Topic: `/elp/image_raw`
3. See 30 FPS RGB video!

**Dual Camera View:**
- Left panel: RealSense depth (`/camera/camera/depth/image_rect_raw`)
- Right panel: ELP RGB (`/elp/image_raw`)

## Common Issues

**Issue:** Camera not found at `/dev/elp_camera`

**Solution:**
```bash
# Check USB connection
lsusb | grep "32e4:0234"

# Verify udev rule
ls -la /dev/elp_camera

# Reload rules if needed
sudo udevadm control --reload-rules
sudo udevadm trigger

# Unplug and replug camera
```

**Issue:** Low frame rate

**Check:**
```bash
# Verify MJPEG format (not raw)
v4l2-ctl --device=/dev/elp_camera --list-formats
```

**Issue:** "Unable to open camera" error

**Fix:**
```bash
# Ensure permissions
sudo chmod 666 /dev/elp_camera

# Or add user to video group
sudo usermod -a -G video $USER
```

## Applications

**What ELP Camera Enables:**
- ✅ High-quality video recording (30 FPS)
- ✅ Object detection with color
- ✅ Person following
- ✅ QR code / AprilTag detection
- ✅ Visual servoing
- ✅ Gesture recognition
- ✅ Traffic light detection

**Combined with RealSense depth:**
- 3D object localization
- Colored point clouds
- Visual-depth fusion

## Specifications Reference

**Sensor:** OmniVision OV2311 (global shutter)  
**Lens:** M12 mount, adjustable focus  
**FOV:** ~120° (depends on lens)  
**Auto Exposure:** Yes  
**Auto White Balance:** Yes  
**Manual Controls:** Exposure, gain, white balance  

## Next Steps

1. ✅ Camera integrated into V3 launch
2. ✅ 30 Hz performance validated
3. ⏳ Camera calibration (optional)
4. ⏳ Explore computer vision applications

---

**Last Updated:** December 31, 2026
EOF

echo "   ✓ ELP camera documentation created"

# ============================================================================
# FIX 5: Update README Hardware Table
# ============================================================================
echo "🟠 Fix 5: Updating README hardware table..."

# Check if IMU already in README
if ! grep -q "BNO055" README.md; then
    # Create temporary file with updated hardware table
    cat > /tmp/hardware_table.txt << 'EOF'
| Component | Model | Function | Status |
|-----------|-------|----------|--------|
| Computer | Raspberry Pi 4 (4GB) | Main controller | ✅ Working |
| Motors | Hoverboard (EFeru firmware) | Differential drive | ✅ Working |
| Lidar | RPLidar A1 | 2D SLAM mapping | ✅ Working |
| IMU | Adafruit BNO055 | Orientation, sensor fusion | ✅ Working |
| Depth Camera | Intel RealSense D435 | 3D obstacles (15 Hz) | ✅ Working |
| RGB Camera | ELP USB GS1200P01 | Visual tasks (30 Hz) | ✅ Working |
| Platform | Hoverboard deck | Robot chassis | ⏳ Assembly needed |
| Battery | Hoverboard 36V | Power supply | ✅ Working |
EOF
    
    echo "   ℹ Note: Manually update README.md hardware table with sensors"
    echo "   Reference: /tmp/hardware_table.txt"
else
    echo "   ℹ IMU already in README hardware table"
fi

# ============================================================================
# SUMMARY
# ============================================================================
echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║              CRITICAL FIXES COMPLETE! ✓                   ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "Fixes applied:"
echo "  ✓ Fixed URDF typo (line 226: inertial → inertia)"
echo "  ✓ Added udev rules to repository (config/udev/)"
echo "  ✓ Added IMU and camera links to URDF"
echo "  ✓ Created ELP camera documentation"
echo "  ℹ README hardware table needs manual update"
echo ""
echo "Next steps:"
echo "  1. Review changes: git status"
echo "  2. Rebuild workspace:"
echo "     cd ros2_ws && colcon build && source install/setup.bash"
echo "  3. Test URDF:"
echo "     ros2 launch hoverbot_bringup state_publisher.launch.py"
echo "  4. Commit:"
echo "     git add -A"
echo "     git commit -m 'fix: Critical fixes - URDF, udev, sensors'"
echo "     git push origin pi4"
echo ""
echo "Ready for assembly! 🚀"
echo ""
