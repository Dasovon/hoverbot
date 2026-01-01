# RViz2 Troubleshooting Guide

## Yellow/Opaque Overlay on Main View

**Symptom:** 
Main viewing area has yellow/colored semi-transparent overlay obscuring the visualization.

**Cause:** 
Odometry covariance visualization is enabled, showing uncertainty ellipse.

**Fix:**
1. In RViz2 left panel, expand "Odometry" display
2. Expand "Covariance" sub-section
3. **Uncheck "Orientation"** checkbox
4. Yellow overlay should disappear immediately

**Alternative causes:**
- Grid display with high cell count (uncheck Grid or reduce Plane Cell Count)
- Map display overlay settings
- Image display overlay enabled

## Other Common RViz2 Issues

### Fixed Frame Errors
- Set Fixed Frame to `map` for SLAM visualization
- Or use `odom` or `base_link` if map frame doesn't exist yet

### Transform Warnings
- Check `ros2 run tf2_tools view_frames` to verify TF tree
- Ensure EKF is running and publishing odom→base_link

### Display Not Updating
- Check topic is publishing: `ros2 topic hz /topic_name`
- Verify Reliability Policy matches publisher (Best Effort vs Reliable)
- For Map: Set Durability Policy to "Transient Local"
