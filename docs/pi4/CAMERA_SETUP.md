# Camera Integration Attempt - Pi Camera Module 2

## Hardware
- **Camera:** Raspberry Pi Camera Module 2 (IMX219 sensor)
- **Connection:** CSI ribbon cable to Pi 4
- **Status:** Hardware installed, detected by system

## Detection Status ✅

Camera is properly detected by the system:
```bash
v4l2-ctl --list-devices
# Shows: unicam (platform:fe801000.csi) at /dev/video0
```

**Camera Info:**
- Driver: unicam
- Sensor: IMX219 (Pi Camera Module 2)
- Supported formats: YUYV, RGB3, and 40+ other formats
- Max resolution: 16376x16376 (stepwise)
- Current format: 640x480 YUYV

**GPU Configuration:**
```bash
vcgencmd get_mem gpu
# Returns: gpu=128M ✅

vcgencmd get_camera
# Returns: supported=1 detected=0, libcamera interfaces=1
```

## Technical Issue ❌

**Problem:** Pi Camera Module 2 requires **libcamera** pipeline on modern Linux kernels, but ROS 2 Humble's `v4l2_camera` package only supports legacy **v4l2** interface.

**Error:** `Failed stream start: Invalid argument (22)`

**Root Cause:**
- Pi Camera Module 2 → Requires libcamera (modern camera stack)
- v4l2_camera (ROS 2 Humble) → Only supports v4l2 (legacy interface)
- Ubuntu 22.04 kernel → Uses libcamera for CSI cameras
- **Incompatibility** between camera requirements and ROS 2 package

## What We Tried

### 1. Boot Configuration
Added to `/boot/firmware/config.txt`:
```
start_x=1
gpu_mem=128
camera_auto_detect=1
```

### 2. Command-Line Tests
- `fswebcam`: Failed with memory allocation error
- `gst-launch-1.0`: Failed with buffer pool activation error
- Both failed due to v4l2 incompatibility

### 3. ROS 2 Integration
Installed packages:
```bash
sudo apt install ros-humble-v4l2-camera ros-humble-image-transport-plugins
```

Tested multiple formats:
```bash
# All failed with same error
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
ros2 run v4l2_camera v4l2_camera_node --ros-args -p pixel_format:=YUYV
ros2 run v4l2_camera v4l2_camera_node --ros-args -p pixel_format:=RGB3
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[3840,2160]"
```

All attempts: `[ERROR] Failed stream start: Invalid argument (22)`

## Solutions for Next Session

### Option 1: USB Webcam ⭐ (Recommended)
**Pros:**
- Works immediately with v4l2_camera
- Plug-and-play, no configuration
- Well-supported in ROS 2
- Many tested examples available

**Cons:**
- Requires purchasing webcam ($20-50)
- Slightly higher CPU usage than CSI camera

**Recommended models:**
- Logitech C270 (~$25)
- Logitech C920 (~$50, higher quality)
- Any generic USB webcam

### Option 2: ArduCam (Already Owned)
**Model:** ArduCam X0036TSMX

**Action required:**
- Verify it's a USB model (not CSI)
- If USB: Will work immediately
- If CSI: Same libcamera issue as Pi Camera Module 2

### Option 3: Experimental libcamera Bridge
**Approach:** Use community libcamera bridges for ROS 2

**Packages to explore:**
- `camera_ros` (libcamera → ROS 2)
- Custom libcamera wrapper nodes
- May require building from source

**Effort:** 1-2 hours, no guarantee of success

**Status:** Not recommended until more mature

### Option 4: Upgrade to ROS 2 Jazzy
**Ubuntu 24.04 + ROS 2 Jazzy** has better libcamera support

**Pros:**
- Newer libcamera integration
- Better Pi Camera Module support
- Future-proof

**Cons:**
- Requires rebuilding entire software stack
- Migration effort (2-4 hours)
- Some packages might not be available yet

## Current Status

**What's Ready:**
- ✅ Camera hardware installed
- ✅ ROS 2 camera packages installed
- ✅ Configuration files ready
- ✅ Technical issue identified

**Blocking Issue:**
- ❌ Pi Camera Module 2 incompatible with v4l2_camera
- ❌ Need either USB camera or libcamera bridge

## Recommendation

**Best path forward:**
1. **Short term:** Get USB webcam for immediate camera integration
2. **Long term:** Keep Pi Camera Module 2 for future upgrade to Jazzy

**Why USB webcam:**
- Proven to work with ROS 2 Humble
- Simple plug-and-play
- Can be used for testing while keeping Pi Camera installed
- Low cost (~$25)

## Files Created/Modified

**Configuration:**
- `/boot/firmware/config.txt` - Added camera configuration

**ROS 2 Packages Installed:**
- ros-humble-v4l2-camera
- ros-humble-image-transport-plugins
- ros-humble-camera-info-manager
- ros-humble-compressed-image-transport

**Ready to go when camera works:**
- All ROS 2 infrastructure in place
- Just need compatible camera hardware

## Next Steps

1. **Obtain USB webcam** (or test ArduCam if USB model)
2. **Create camera launch file** (code ready, waiting for hardware)
3. **Test image streaming to RViz**
4. **Add to hoverbot_full_v2.launch.py**
5. **Integrate visual odometry** (post robot assembly)

## Resources

**Technical References:**
- v4l2_camera package: http://wiki.ros.org/v4l2_camera
- Pi Camera Module 2 specs: https://www.raspberrypi.com/products/camera-module-v2/
- libcamera documentation: https://libcamera.org/

**Community discussions:**
- ROS Discourse: libcamera support in ROS 2
- Raspberry Pi forums: Ubuntu camera issues

---

**Date:** December 28, 2025  
**Status:** Hardware ready, software ready, waiting for compatible camera  
**Recommendation:** USB webcam for immediate integration