#!/usr/bin/env python3
"""
Test Script for HoverBot Serial Protocol

Standalone test to verify serial communication with the hoverboard
before running the full ROS 2 driver. This helps isolate issues.

Usage:
    python3 test_serial_protocol.py [port] [baud]
    
Example:
    python3 test_serial_protocol.py /dev/ttyAMA0 115200
"""

import sys
import time
import struct


def test_serial_connection(port='/dev/ttyAMA0', baudrate=115200):
    """
    Test serial connection to hoverboard.
    
    Sends zero commands and checks for valid feedback.
    """
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed")
        print("Install with: pip install pyserial")
        return False
    
    print(f"\n{'='*60}")
    print("HoverBot Serial Protocol Test")
    print(f"{'='*60}\n")
    
    print(f"Port: {port}")
    print(f"Baud: {baudrate}")
    print(f"\nAttempting to connect...")
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        print("✓ Serial port opened successfully")
    except serial.SerialException as e:
        print(f"✗ Failed to open serial port: {e}")
        print("\nTroubleshooting:")
        print("  1. Check port exists: ls -l /dev/ttyAMA*")
        print("  2. Check permissions: sudo chmod 666 /dev/ttyAMA0")
        print("  3. Check UART enabled: ls /dev | grep ttyAMA")
        return False
    
    # Flush buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    print(f"\nSending test commands (zero velocity)...")
    
    # Send zero commands for 3 seconds
    START_FRAME = 0xABCD
    tx_count = 0
    rx_count = 0
    errors = 0
    
    start_time = time.time()
    duration = 3.0  # seconds
    
    while time.time() - start_time < duration:
        # Send zero command
        steer = 0
        speed = 0
        checksum = (START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
        
        packet = struct.pack('<HhhH', START_FRAME, steer, speed, checksum)
        bytes_written = ser.write(packet)
        
        if bytes_written == 8:
            tx_count += 1
        
        # Try to read feedback
        if ser.in_waiting >= 18:
            data = ser.read(18)
            if len(data) == 18:
                try:
                    # Unpack feedback
                    unpacked = struct.unpack('<HhhhhhhHH', data)
                    start, cmd1, cmd2, speed_r, speed_l, bat_v, temp, led, rx_checksum = unpacked
                    
                    if start == START_FRAME:
                        # Validate checksum
                        calc_checksum = start
                        for value in unpacked[1:-1]:
                            calc_checksum ^= (value & 0xFFFF)
                        calc_checksum &= 0xFFFF
                        
                        if calc_checksum == rx_checksum:
                            rx_count += 1
                            
                            # Print first feedback packet details
                            if rx_count == 1:
                                print(f"\n✓ Valid feedback received!")
                                print(f"  Battery: {bat_v/100.0:.2f} V")
                                print(f"  Temperature: {temp/10.0:.1f} °C")
                                print(f"  Speed Left: {speed_l} RPM")
                                print(f"  Speed Right: {speed_r} RPM")
                        else:
                            errors += 1
                except struct.error:
                    errors += 1
        
        time.sleep(0.02)  # 50Hz
    
    ser.close()
    
    print(f"\n{'='*60}")
    print("Test Results:")
    print(f"{'='*60}")
    print(f"  Commands sent:     {tx_count}")
    print(f"  Feedback received: {rx_count}")
    print(f"  Errors:            {errors}")
    
    if rx_count > 0:
        success_rate = (rx_count / tx_count) * 100 if tx_count > 0 else 0
        print(f"  Success rate:      {success_rate:.1f}%")
        
        if success_rate > 80:
            print(f"\n✓ PASS - Communication working well!")
            print(f"  Your hoverboard is responding correctly.")
            print(f"  Ready to run ROS 2 driver node.")
            return True
        else:
            print(f"\n⚠ PARTIAL - Communication working but with errors")
            print(f"  Check wiring and reduce electrical noise")
            return True
    else:
        print(f"\n✗ FAIL - No valid feedback received")
        print(f"\nTroubleshooting:")
        print(f"  1. Verify firmware has CONTROL_SERIAL_USART3 enabled")
        print(f"  2. Check wiring: Pi TX→Hoverboard RX, Pi RX→Hoverboard TX")
        print(f"  3. Confirm battery connected and hoverboard powered on")
        print(f"  4. Try the other sensor cable (USART2 vs USART3)")
        print(f"  5. Check for continuous beeping (indicates timeout)")
        return False


def main():
    """Main entry point."""
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyAMA0'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    success = test_serial_connection(port, baud)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
