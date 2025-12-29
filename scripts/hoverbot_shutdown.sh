#!/bin/bash
###############################################################################
# HoverBot Clean Shutdown Script
#
# Stops all HoverBot ROS 2 nodes cleanly
# Sends zero velocity command before stopping driver
#
# Usage:
#   ./hoverbot_shutdown.sh
###############################################################################

set -e

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         HoverBot Shutdown - Stopping All Nodes            ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Send stop command to robot (safety)
echo "[1/3] Sending stop command to robot..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 || true
sleep 0.5

# Kill all ROS 2 nodes
echo "[2/3] Stopping all ROS 2 nodes..."
killall -9 ros2 2>/dev/null || true
killall -9 rplidarNode 2>/dev/null || true
killall -9 async_slam_toolbox_node 2>/dev/null || true
killall -9 ekf_node 2>/dev/null || true
killall -9 static_transform_publisher 2>/dev/null || true
sleep 1

# Clean up any zombie processes
echo "[3/3] Cleaning up..."
killall -9 python3 2>/dev/null || true

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                  Shutdown Complete ✓                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
