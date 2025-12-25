
# Raspberry Pi 4 Configuration

## Hardware
- Raspberry Pi 4 (4GB or 8GB)
- IP: 192.168.86.20
- Ubuntu 22.04 LTS Server
- ROS 2 Humble Hawksbill

## Status
✅ Driver working
✅ Serial communication validated
✅ Teleop control functional
✅ Ready for SLAM and Nav2

## UART Wiring (Right Sideboard - USART3)
- Green wire → Pi GPIO14 (TX, Pin 8)
- Blue wire → Pi GPIO15 (RX, Pin 10)
- Black wire → Pi GND (Pin 6)
- Red wire → NOT CONNECTED (15V!)

## Notes
- Wire colors were swapped from expected
- Loopback test confirmed Pi RX working
- 99.3% communication success rate
