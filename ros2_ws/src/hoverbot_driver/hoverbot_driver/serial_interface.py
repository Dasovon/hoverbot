"""
Serial Interface for EFeru Hoverboard Controller

Implements the USART protocol for bidirectional communication:
- Command packets: 8 bytes (0xABCD + steer + speed + checksum)
- Feedback packets: 18 bytes (telemetry data at 100Hz)

Protocol validated with BOARD_VARIANT=1, TANK_STEERING enabled.
"""

import serial
import struct
import time
from typing import Optional, Tuple, Dict
from dataclasses import dataclass


@dataclass
class HoverboardFeedback:
    """
    Telemetry data received from hoverboard at 100Hz.
    All values are raw from firmware before unit conversion.
    """
    cmd1: int  # Processed command 1 (steer/brake)
    cmd2: int  # Processed command 2 (speed/throttle)
    speed_r_rpm: int  # Right wheel speed in RPM
    speed_l_rpm: int  # Left wheel speed in RPM
    bat_voltage: int  # Battery voltage × 100 (e.g., 3600 = 36.00V)
    board_temp: int  # Temperature × 10 (e.g., 235 = 23.5°C)
    led: int  # LED control word (unused)
    timestamp: float  # System time when received


class HoverboardSerialInterface:
    """
    Low-level serial communication with EFeru hoverboard firmware.
    
    Handles:
    - Command transmission with XOR checksum
    - Feedback reception and validation
    - Protocol error detection
    - Connection management
    """
    
    # Protocol constants
    START_FRAME = 0xABCD
    CMD_PACKET_SIZE = 8  # bytes
    FEEDBACK_PACKET_SIZE = 18  # bytes
    
    # Command limits (firmware range: -1000 to +1000)
    CMD_MIN = -1000
    CMD_MAX = 1000
    
    def __init__(self, port: str = '/dev/ttyAMA0', baudrate: int = 115200, timeout: float = 0.1):
        """
        Initialize serial interface.
        
        Args:
            port: Serial device path (e.g., '/dev/ttyAMA0')
            baudrate: Communication speed (default 115200)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        
        # Statistics
        self.tx_count = 0
        self.rx_count = 0
        self.checksum_errors = 0
        self.framing_errors = 0
        
    def connect(self) -> bool:
        """
        Open serial connection to hoverboard.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            # Flush any stale data
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            return True
        except serial.SerialException as e:
            print(f"Failed to open serial port {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
    
    def is_connected(self) -> bool:
        """Check if serial connection is active."""
        return self.serial is not None and self.serial.is_open
    
    @staticmethod
    def _clamp(value: int, min_val: int, max_val: int) -> int:
        """Clamp value to valid range."""
        return max(min_val, min(max_val, value))
    
    @staticmethod
    def _calculate_checksum(start: int, steer: int, speed: int) -> int:
        """
        Calculate XOR checksum for command packet.
        
        Args:
            start: Start frame (0xABCD)
            steer: Steer command (int16)
            speed: Speed command (int16)
            
        Returns:
            Checksum as uint16
        """
        # Convert signed int16 to unsigned for XOR
        steer_u = steer & 0xFFFF
        speed_u = speed & 0xFFFF
        return (start ^ steer_u ^ speed_u) & 0xFFFF
    
    def send_command(self, steer: int, speed: int) -> bool:
        """
        Send motor command to hoverboard.
        
        With TANK_STEERING enabled:
        - steer: Left wheel command (-1000 to +1000)
        - speed: Right wheel command (-1000 to +1000)
        
        Args:
            steer: Left wheel command
            speed: Right wheel command
            
        Returns:
            True if transmission successful, False otherwise
        """
        if not self.is_connected():
            return False
        
        # Clamp to safe limits
        steer = self._clamp(steer, self.CMD_MIN, self.CMD_MAX)
        speed = self._clamp(speed, self.CMD_MIN, self.CMD_MAX)
        
        # Calculate checksum
        checksum = self._calculate_checksum(self.START_FRAME, steer, speed)
        
        # Pack command (little-endian: <HhhH)
        # H = unsigned short (uint16), h = signed short (int16)
        packet = struct.pack('<HhhH', self.START_FRAME, steer, speed, checksum)
        
        try:
            bytes_written = self.serial.write(packet)
            self.tx_count += 1
            return bytes_written == self.CMD_PACKET_SIZE
        except serial.SerialException as e:
            print(f"Serial write error: {e}")
            return False
    
    def read_feedback(self) -> Optional[HoverboardFeedback]:
        """
        Read telemetry feedback from hoverboard.
        
        Feedback is sent at 100Hz by firmware. This is a non-blocking read.
        
        Returns:
            HoverboardFeedback object if valid packet received, None otherwise
        """
        if not self.is_connected():
            return None
        
        try:
            # Check if data available
            if self.serial.in_waiting < self.FEEDBACK_PACKET_SIZE:
                return None
            
            # Read packet
            data = self.serial.read(self.FEEDBACK_PACKET_SIZE)
            if len(data) != self.FEEDBACK_PACKET_SIZE:
                self.framing_errors += 1
                return None
            
            # Unpack (little-endian: <HhhhhhhHH)
            # Format: start, cmd1, cmd2, speedR, speedL, batVolt, temp, led, checksum
            unpacked = struct.unpack('<HhhhhhhHH', data)
            
            start, cmd1, cmd2, speed_r, speed_l, bat_v, temp, led, rx_checksum = unpacked
            
            # Validate start frame
            if start != self.START_FRAME:
                self.framing_errors += 1
                return None
            
            # Validate checksum (XOR of all fields except checksum itself)
            calc_checksum = start
            for value in unpacked[1:-1]:  # Skip start (already included) and checksum
                calc_checksum ^= (value & 0xFFFF)
            calc_checksum &= 0xFFFF
            
            if calc_checksum != rx_checksum:
                self.checksum_errors += 1
                return None
            
            # Valid packet
            self.rx_count += 1
            return HoverboardFeedback(
                cmd1=cmd1,
                cmd2=cmd2,
                speed_r_rpm=speed_r,
                speed_l_rpm=speed_l,
                bat_voltage=bat_v,
                board_temp=temp,
                led=led,
                timestamp=time.time()
            )
            
        except serial.SerialException as e:
            print(f"Serial read error: {e}")
            return None
    
    def get_stats(self) -> Dict[str, int]:
        """
        Get communication statistics.
        
        Returns:
            Dictionary with tx_count, rx_count, checksum_errors, framing_errors
        """
        return {
            'tx_count': self.tx_count,
            'rx_count': self.rx_count,
            'checksum_errors': self.checksum_errors,
            'framing_errors': self.framing_errors
        }
    
    def reset_stats(self):
        """Reset communication statistics."""
        self.tx_count = 0
        self.rx_count = 0
        self.checksum_errors = 0
        self.framing_errors = 0
