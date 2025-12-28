# Launch File Fix - RPLidar Buffer Overflow Solved

## Problem
RPLidar crashed with buffer overflow when launched via combined launch files.

## Solution
Use official `rplidar_a1_launch.py` with increased delays:
- Driver: Immediate start
- TF Static: 2 second delay
- RPLidar: 6 second delay (key fix - was 5s, increased to 6s)
- SLAM: 10 second delay

## Working Launch File
`hoverbot_full_v2.launch.py` - Uses official RPLidar launch with proper timing

## Updated Tmux Startup
`scripts/hoverbot_startup.sh` - Now uses single launch file instead of 4 separate components

## Result
✅ No buffer overflow
✅ No timeout errors
✅ Clean single-pane launch
✅ All components start reliably

## Testing Confirmed
- Date: December 28, 2025
- RPLidar: /dev/ttyUSB1 (CP2102 USB bridge)
- All nodes running without duplicates
- Map publishing at 0.1 Hz
- System stable

## Usage
```bash
cd ~/hoverbot/scripts
./hoverbot_startup.sh
```

Simple, reliable, one-command startup achieved!