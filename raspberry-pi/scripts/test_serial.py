#!/usr/bin/env python3
"""
Hoverboard Serial Control Test Script

Tests serial communication with EFeru FOC firmware on STM32F103 hoverboard controller.

Protocol:
    - 8 byte packets (little-endian)
    - Start: 0xABCD
    - Steer: int16 (-1000 to +1000)
    - Speed: int16 (-1000 to +1000)
    - Checksum: start XOR steer XOR speed

Usage:
    python3 test_serial.py

Requirements:
    pip install pyserial
"""

import serial
import struct
import time
import sys
import argparse


class HoverboardController:
    """Interface for EFeru FOC hoverboard controller via UART"""
    
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        """
        Initialize serial connection
        
        Args:
            port: Serial port device (default: /dev/ttyAMA0 for Pi UART)
            baudrate: Communication speed (default: 115200)
        """
        self.START_FRAME = 0xABCD
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
    def connect(self):
        """Open serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to open {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial port"""
        if self.ser and self.ser.is_open:
            # Send stop commands before closing
            for _ in range(10):
                self.send_command(0, 0)
                time.sleep(0.02)
            self.ser.close()
            print("✓ Serial port closed")
    
    def send_command(self, steer, speed):
        """
        Send motor command to hoverboard
        
        Args:
            steer: Steering value -1000 to +1000 (right wheel in TANK_STEERING)
            speed: Speed value -1000 to +1000 (left wheel in TANK_STEERING)
        
        Returns:
            bool: True if sent successfully
        """
        if not self.ser or not self.ser.is_open:
            return False
        
        # Clamp values
        steer = max(-1000, min(1000, steer))
        speed = max(-1000, min(1000, speed))
        
        # Calculate checksum
        checksum = self.START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
        
        # Pack packet (little-endian: < prefix)
        # H = uint16, h = int16
        packet = struct.pack('<HhhH', self.START_FRAME, steer, speed, checksum)
        
        try:
            self.ser.write(packet)
            return True
        except serial.SerialException as e:
            print(f"✗ Send failed: {e}")
            return False
    
    def read_feedback(self):
        """
        Read telemetry feedback from hoverboard
        
        Returns:
            dict: Feedback data or None if unavailable
        """
        if not self.ser or not self.ser.is_open:
            return None
        
        try:
            if self.ser.in_waiting >= 18:  # Full feedback packet
                data = self.ser.read(18)
                if len(data) == 18:
                    # Unpack feedback packet
                    values = struct.unpack('<HhhhhhhHH', data)
                    
                    if values[0] == self.START_FRAME:
                        return {
                            'start': values[0],
                            'cmd1': values[1],
                            'cmd2': values[2],
                            'speedR': values[3],
                            'speedL': values[4],
                            'batVoltage': values[5] / 100.0,  # Convert to volts
                            'boardTemp': values[6] / 10.0,     # Convert to °C
                            'cmdLed': values[7],
                            'checksum': values[8]
                        }
        except Exception as e:
            print(f"✗ Read failed: {e}")
        
        return None


def test_connection(controller):
    """Test basic serial communication"""
    print("\n=== Connection Test ===")
    print("Sending zero commands for 2 seconds...")
    print("Expected: Beeping should stop\n")
    
    start_time = time.time()
    count = 0
    
    while time.time() - start_time < 2.0:
        controller.send_command(0, 0)
        count += 1
        time.sleep(0.02)  # 50 Hz
    
    print(f"Sent {count} packets")
    print("Did the beeping stop? (y/n): ", end='')
    response = input().strip().lower()
    
    return response == 'y'


def test_motors(controller):
    """Test motor movement"""
    print("\n=== Motor Movement Test ===")
    print("⚠️  WARNING: Wheels must be OFF THE GROUND!")
    print("Press Enter when ready, or Ctrl+C to skip...")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\nSkipping motor test")
        return
    
    tests = [
        ("Forward (speed=100)", 100, 100, 2.0),
        ("Backward (speed=-100)", -100, -100, 2.0),
        ("Turn right (steer=200)", 200, 0, 2.0),
        ("Turn left (steer=-200)", -200, 0, 2.0),
    ]
    
    for name, steer, speed, duration in tests:
        print(f"\nTest: {name}")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            controller.send_command(steer, speed)
            time.sleep(0.02)
        
        # Stop between tests
        print("Stopping...")
        for _ in range(25):
            controller.send_command(0, 0)
            time.sleep(0.02)
        
        time.sleep(0.5)
    
    print("\n✓ Motor tests complete")


def test_feedback(controller):
    """Test telemetry feedback"""
    print("\n=== Feedback Test ===")
    print("Reading telemetry for 5 seconds...\n")
    
    start_time = time.time()
    feedback_count = 0
    
    while time.time() - start_time < 5.0:
        controller.send_command(0, 0)
        
        feedback = controller.read_feedback()
        if feedback:
            feedback_count += 1
            if feedback_count % 10 == 0:  # Print every 10th packet
                print(f"Battery: {feedback['batVoltage']:.2f}V  "
                      f"Temp: {feedback['boardTemp']:.1f}°C  "
                      f"Speed R/L: {feedback['speedR']}/{feedback['speedL']}")
        
        time.sleep(0.02)
    
    print(f"\n✓ Received {feedback_count} feedback packets")


def interactive_control(controller):
    """Interactive keyboard control"""
    print("\n=== Interactive Control ===")
    print("Commands:")
    print("  w/s - Forward/Backward")
    print("  a/d - Turn left/right")
    print("  x   - Stop")
    print("  q   - Quit")
    print("\n⚠️  Wheels must be OFF THE GROUND!")
    print("\nPress Enter to start, Ctrl+C to skip...")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\nSkipping interactive control")
        return
    
    import tty
    import termios
    
    speed_step = 100
    steer = 0
    speed = 0
    
    # Save terminal settings
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setcbreak(fd)
        print("\nControl active (q to quit)...\n")
        
        while True:
            # Send current command at 50Hz
            controller.send_command(steer, speed)
            
            # Check for key press (non-blocking)
            import select
            if select.select([sys.stdin], [], [], 0.02)[0]:
                key = sys.stdin.read(1)
                
                if key == 'q':
                    break
                elif key == 'w':
                    speed = min(1000, speed + speed_step)
                    print(f"Speed: {speed}")
                elif key == 's':
                    speed = max(-1000, speed - speed_step)
                    print(f"Speed: {speed}")
                elif key == 'a':
                    steer = max(-1000, steer - speed_step)
                    print(f"Steer: {steer}")
                elif key == 'd':
                    steer = min(1000, steer + speed_step)
                    print(f"Steer: {steer}")
                elif key == 'x':
                    steer = 0
                    speed = 0
                    print("STOP")
            
            time.sleep(0.02)
    
    finally:
        # Restore terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        
        # Send stop commands
        print("\nStopping motors...")
        for _ in range(25):
            controller.send_command(0, 0)
            time.sleep(0.02)


def main():
    """Main test program"""
    parser = argparse.ArgumentParser(description='Test hoverboard serial communication')
    parser.add_argument('--port', default='/dev/ttyAMA0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--skip-motors', action='store_true', help='Skip motor tests')
    args = parser.parse_args()
    
    print("=" * 50)
    print("Hoverboard Serial Communication Test")
    print("=" * 50)
    
    controller = HoverboardController(port=args.port, baudrate=args.baud)
    
    if not controller.connect():
        return 1
    
    try:
        # Run tests
        if not test_connection(controller):
            print("\n⚠️  Connection test failed - check wiring and firmware")
            return 1
        
        test_feedback(controller)
        
        if not args.skip_motors:
            test_motors(controller)
            interactive_control(controller)
        
        print("\n" + "=" * 50)
        print("✓ All tests complete!")
        print("=" * 50)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        controller.disconnect()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
