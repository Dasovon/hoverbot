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
