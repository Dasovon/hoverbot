#!/bin/bash
################################################################################
# HoverBot Startup Script - Basic Version
# Usage: ./hoverbot_startup.sh
################################################################################

SESSION_NAME="hoverbot"
PI_HOST="ryan@192.168.86.20"
ROS_SETUP="source /opt/ros/humble/setup.bash; source ~/hoverbot/ros2_ws/install/setup.bash"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}Starting HoverBot tmux session...${NC}"

# Kill existing session if it exists
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create new session with first window (4 panes)
tmux new-session -d -s "$SESSION_NAME" -n "robot"

# Split into 4 panes (2x2 grid)
tmux split-window -h -t "$SESSION_NAME:0"
tmux split-window -v -t "$SESSION_NAME:0.0"
tmux split-window -v -t "$SESSION_NAME:0.1"

# Pane layout:
# 0.0 = top-left     (DRIVER)
# 0.1 = bottom-left  (RPLIDAR)
# 0.2 = top-right    (TF_STATIC)
# 0.3 = bottom-right (SLAM)

# Set pane titles
tmux select-pane -t "$SESSION_NAME:0.0" -T "DRIVER"
tmux select-pane -t "$SESSION_NAME:0.1" -T "RPLIDAR"
tmux select-pane -t "$SESSION_NAME:0.2" -T "TF_STATIC"
tmux select-pane -t "$SESSION_NAME:0.3" -T "SLAM"

# Pane 0: Driver (start first)
tmux send-keys -t "$SESSION_NAME:0.0" "ssh $PI_HOST" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0.0" "$ROS_SETUP" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0.0" "ros2 launch hoverbot_driver hoverbot_driver.launch.py" C-m

# Wait for driver to start
echo "Waiting for driver to start (3 seconds)..."
sleep 3

# Pane 2: Static Transform
tmux send-keys -t "$SESSION_NAME:0.2" "ssh $PI_HOST" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0.2" "$ROS_SETUP" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0.2" "ros2 run tf2_ros static_transform_publisher --x 0.1 --y 0 --z 0.1 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id laser" C-m

# Wait for TF to start
echo "Waiting for TF publisher (2 seconds)..."
sleep 2

# Pane 1: RPLidar
tmux send-keys -t "$SESSION_NAME:0.1" "ssh $PI_HOST" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0.1" "$ROS_SETUP" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0.1" "ros2 launch rplidar_ros rplidar_a1_launch.py" C-m

# Wait for RPLidar
echo "Waiting for RPLidar (5 seconds)..."
sleep 5

# Pane 3: SLAM
tmux send-keys -t "$SESSION_NAME:0.3" "ssh $PI_HOST" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0.3" "$ROS_SETUP" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0.3" "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/ryan/hoverbot/ros2_ws/install/hoverbot_bringup/share/hoverbot_bringup/config/slam_toolbox_params.yaml use_sim_time:=false" C-m

# Create second window for teleop
tmux new-window -t "$SESSION_NAME:1" -n "teleop"
tmux send-keys -t "$SESSION_NAME:1" "ssh $PI_HOST" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:1" "$ROS_SETUP" C-m
sleep 1
echo ""
echo -e "${GREEN}HoverBot started!${NC}"
echo ""
echo "Tmux session: $SESSION_NAME"
echo ""
echo "Layout:"
echo "  Window 0 (robot):"
echo "    ┌──────────┬──────────┐"
echo "    │  DRIVER  │ TF_STATIC│"
echo "    ├──────────┼──────────┤"
echo "    │ RPLIDAR  │   SLAM   │"
echo "    └──────────┴──────────┘"
echo ""
echo "  Window 1 (teleop): Ready to start teleop"
echo ""
echo "Commands:"
echo "  Attach:  tmux attach -t $SESSION_NAME"
echo "  Detach:  Ctrl+b, then d"
echo "  Switch:  Ctrl+b, then 0 or 1"
echo "  Kill:    tmux kill-session -t $SESSION_NAME"
echo ""
echo "Attaching in 3 seconds..."
sleep 3

# Attach to session (window 0)
tmux attach -t "$SESSION_NAME:0"
