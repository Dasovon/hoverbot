#!/bin/bash
################################################################################
# HoverBot Shutdown Script
# Usage: ./hoverbot_shutdown.sh
################################################################################

SESSION_NAME="hoverbot"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}Stopping HoverBot...${NC}"

# Check if session exists
if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo -e "${RED}HoverBot is not running${NC}"
    exit 1
fi

# Send Ctrl+C to gracefully stop the launch
echo "Sending shutdown signal..."
tmux send-keys -t "$SESSION_NAME:0" C-c

# Wait for graceful shutdown
sleep 3

# Kill the session
tmux kill-session -t "$SESSION_NAME" 2>/dev/null

echo -e "${GREEN}HoverBot stopped${NC}"