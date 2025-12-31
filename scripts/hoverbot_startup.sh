#!/bin/bash
################################################################################
# HoverBot Startup Script - Simplified with Single Launch File
# Usage: ./hoverbot_startup.sh
################################################################################

SESSION_NAME="hoverbot"
PI_HOST="ryan@192.168.86.20"
ROS_SETUP="source /opt/ros/humble/setup.bash; source ~/hoverbot/ros2_ws/install/setup.bash"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}Starting HoverBot tmux session (single launch file)...${NC}"

# Kill existing session if it exists
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create new session with main window
tmux new-session -d -s "$SESSION_NAME" -n "robot"

# Main pane: Launch all components via single launch file
tmux send-keys -t "$SESSION_NAME:0" "ssh $PI_HOST" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0" "$ROS_SETUP" C-m
sleep 1
tmux send-keys -t "$SESSION_NAME:0" "ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py" C-m

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
echo "  Window 0 (robot): All components via single launch file"
echo "  Window 1 (teleop): Ready to start teleop"
echo ""
echo "Commands:"
echo "  Attach:  tmux attach -t $SESSION_NAME"
echo "  Detach:  Ctrl+b, then d"
echo "  Switch:  Ctrl+b, then 0 or 1"
echo "  Kill:    tmux kill-session -t $SESSION_NAME"
echo ""
echo "Components starting (will be ready in ~12 seconds):"
echo "  - Driver (immediate)"
echo "  - TF Static (2 seconds)"
echo "  - RPLidar (6 seconds)"
echo "  - SLAM (10 seconds)"
echo ""
echo "Attaching in 3 seconds..."
sleep 3

# Attach to session (window 0)
tmux attach -t "$SESSION_NAME:0"