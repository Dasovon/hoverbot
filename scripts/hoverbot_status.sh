#!/bin/bash
################################################################################
# HoverBot Status Check
# Usage: ./hoverbot_status.sh
################################################################################

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== HoverBot Status ===${NC}\n"

# Check tmux session
if tmux has-session -t hoverbot 2>/dev/null; then
    echo -e "${GREEN}✅ Tmux Session: Running${NC}"
else
    echo -e "${RED}❌ Tmux Session: Not running${NC}"
    echo -e "\nStart with: ${YELLOW}cd ~/hoverbot/scripts && ./hoverbot_startup.sh${NC}"
    exit 1
fi

# Source ROS (suppress hoverbot_description warning)
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash 2>/dev/null

# Check ROS nodes
echo -e "\n${BLUE}ROS 2 Nodes:${NC}"
nodes=$(ros2 node list 2>/dev/null)
node_count=$(echo "$nodes" | grep -v "^$" | wc -l)

if [ $node_count -gt 0 ]; then
    echo -e "${GREEN}✅ $node_count nodes running:${NC}"
    echo "$nodes" | sed 's/^/   /'
else
    echo -e "${RED}❌ No ROS nodes found${NC}"
fi

# Check key topics
echo -e "\n${BLUE}Topic Health:${NC}"

# Odometry
if ros2 topic info /odom &>/dev/null; then
    echo -e "${GREEN}✅ Odometry: Topic exists${NC}"
else
    echo -e "${RED}❌ Odometry: Not found${NC}"
fi

# Lidar
if ros2 topic info /scan &>/dev/null; then
    echo -e "${GREEN}✅ Lidar Scan: Topic exists${NC}"
else
    echo -e "${RED}❌ Lidar Scan: Not found${NC}"
fi

# Map
if ros2 topic info /map &>/dev/null; then
    echo -e "${GREEN}✅ SLAM Map: Topic exists${NC}"
else
    echo -e "${RED}❌ SLAM Map: Not found${NC}"
fi

# Show topic rates
echo -e "\n${BLUE}Publishing Rates (live check):${NC}"
timeout 3 ros2 topic hz /odom 2>/dev/null | head -1 | grep "average" && echo "" || echo -e "${YELLOW}  /odom: Checking...${NC}"
timeout 3 ros2 topic hz /scan 2>/dev/null | head -1 | grep "average" && echo "" || echo -e "${YELLOW}  /scan: Checking...${NC}"