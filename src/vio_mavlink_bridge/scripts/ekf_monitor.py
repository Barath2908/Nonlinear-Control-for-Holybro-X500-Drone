#!/usr/bin/env python3
"""
EKF3 Status Monitor for ArduPilot

Monitors EKF3 convergence, innovation values, and vision position messages
for debugging GPS-denied navigation.

Author: Barath Kumar JK
Date: 2025

Usage:
    ros2 run vio_mavlink_bridge ekf_monitor.py --port /dev/ttyACM0
"""

import argparse
import sys
import time
from datetime import datetime
from collections import deque

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except ImportError:
    PYMAVLINK_AVAILABLE = False
    print("Error: pymavlink not installed. Install with: pip install pymavlink")


class EKFMonitor:
    """Monitor EKF3 status and vision position messages."""
    
    def __init__(self, connection_string: str, baudrate: int = 921600):
        """
        Initialize EKF monitor.
        
        Args:
            connection_string: Serial port or UDP connection
            baudrate: Serial baud rate
        """
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.mav = None
        
        # Stats
        self.vision_msg_count = 0
        self.last_vision_time = None
        self.vision_rate_history = deque(maxlen=30)
        
        self.ekf_status = {
            'velocity_variance': 0.0,
            'pos_horiz_variance': 0.0,
            'pos_vert_variance': 0.0,
            'compass_variance': 0.0,
            'terrain_alt_variance': 0.0,
            'flags': 0,
        }
        
        self.position = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
        }
        
    def connect(self) -> bool:
        """Connect to flight controller."""
        try:
            print(f"Connecting to {self.connection_string}...")
            self.mav = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate,
            )
            
            print("Waiting for heartbeat...")
            self.mav.wait_heartbeat(timeout=10)
            print(f"Connected to system {self.mav.target_system}")
            
            # Request message streams
            self.request_message_intervals()
            
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
            
    def request_message_intervals(self):
        """Request specific message streams."""
        messages = [
            (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),  # 2 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, 10),  # 10 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5),  # 5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5),  # 5 Hz
        ]
        
        for msg_id, rate_hz in messages:
            interval_us = int(1e6 / rate_hz)
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0, 0, 0, 0, 0,
            )
            
    def process_message(self, msg):
        """Process incoming MAVLink message."""
        msg_type = msg.get_type()
        
        if msg_type == 'EKF_STATUS_REPORT':
            self.ekf_status['velocity_variance'] = msg.velocity_variance
            self.ekf_status['pos_horiz_variance'] = msg.pos_horiz_variance
            self.ekf_status['pos_vert_variance'] = msg.pos_vert_variance
            self.ekf_status['compass_variance'] = msg.compass_variance
            self.ekf_status['terrain_alt_variance'] = msg.terrain_alt_variance
            self.ekf_status['flags'] = msg.flags
            
        elif msg_type == 'VISION_POSITION_ESTIMATE':
            self.vision_msg_count += 1
            now = time.time()
            
            if self.last_vision_time is not None:
                dt = now - self.last_vision_time
                if dt > 0:
                    self.vision_rate_history.append(1.0 / dt)
                    
            self.last_vision_time = now
            
            self.position['x'] = msg.x
            self.position['y'] = msg.y
            self.position['z'] = msg.z
            self.position['roll'] = msg.roll
            self.position['pitch'] = msg.pitch
            self.position['yaw'] = msg.yaw
            
        elif msg_type == 'STATUSTEXT':
            severity = ['EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR',
                       'WARNING', 'NOTICE', 'INFO', 'DEBUG'][msg.severity]
            print(f"[{severity}] {msg.text}")
            
    def get_ekf_status_string(self) -> str:
        """Get human-readable EKF status."""
        flags = self.ekf_status['flags']
        
        status_bits = [
            (0x01, 'ATT'),      # Attitude estimate OK
            (0x02, 'VEL_H'),    # Horizontal velocity OK
            (0x04, 'VEL_V'),    # Vertical velocity OK
            (0x08, 'POS_H'),    # Horizontal position OK
            (0x10, 'POS_V'),    # Vertical position OK
            (0x20, 'CONST_POS'),  # Constant position mode
            (0x40, 'PRED_H'),   # Predict horizontal position OK
            (0x80, 'PRED_V'),   # Predict vertical position OK
        ]
        
        active = []
        for bit, name in status_bits:
            if flags & bit:
                active.append(name)
                
        return ' '.join(active) if active else 'NONE'
        
    def is_ekf_converged(self) -> bool:
        """Check if EKF has converged."""
        # Check variance thresholds
        if self.ekf_status['pos_horiz_variance'] > 1.0:
            return False
        if self.ekf_status['velocity_variance'] > 1.0:
            return False
            
        # Check required flags (attitude + position)
        required_flags = 0x01 | 0x08 | 0x10  # ATT + POS_H + POS_V
        if (self.ekf_status['flags'] & required_flags) != required_flags:
            return False
            
        return True
        
    def get_vision_rate(self) -> float:
        """Get current vision message rate."""
        if len(self.vision_rate_history) > 0:
            return sum(self.vision_rate_history) / len(self.vision_rate_history)
        return 0.0
        
    def print_status(self):
        """Print current status."""
        # Clear screen
        print("\033[2J\033[H", end="")
        
        print("=" * 60)
        print("         EKF3 Monitor - GPS-Denied Navigation")
        print("=" * 60)
        print()
        
        # EKF Status
        converged = self.is_ekf_converged()
        status_color = '\033[92m' if converged else '\033[91m'  # Green or red
        reset_color = '\033[0m'
        
        print(f"EKF Status: {status_color}{'CONVERGED' if converged else 'NOT CONVERGED'}{reset_color}")
        print(f"  Flags: {self.get_ekf_status_string()}")
        print(f"  Position H variance: {self.ekf_status['pos_horiz_variance']:.4f}")
        print(f"  Position V variance: {self.ekf_status['pos_vert_variance']:.4f}")
        print(f"  Velocity variance:   {self.ekf_status['velocity_variance']:.4f}")
        print(f"  Compass variance:    {self.ekf_status['compass_variance']:.4f}")
        print()
        
        # Vision Status
        vision_rate = self.get_vision_rate()
        rate_ok = 20.0 <= vision_rate <= 50.0
        rate_color = '\033[92m' if rate_ok else '\033[93m'  # Green or yellow
        
        print(f"Vision Messages: {self.vision_msg_count}")
        print(f"  Rate: {rate_color}{vision_rate:.1f} Hz{reset_color}")
        print()
        
        # Position
        print("Vision Position (NED):")
        print(f"  X: {self.position['x']:8.3f} m  (North)")
        print(f"  Y: {self.position['y']:8.3f} m  (East)")
        print(f"  Z: {self.position['z']:8.3f} m  (Down)")
        print()
        print("Vision Attitude:")
        print(f"  Roll:  {self.position['roll']:7.2f} rad ({self.position['roll'] * 57.3:6.1f}°)")
        print(f"  Pitch: {self.position['pitch']:7.2f} rad ({self.position['pitch'] * 57.3:6.1f}°)")
        print(f"  Yaw:   {self.position['yaw']:7.2f} rad ({self.position['yaw'] * 57.3:6.1f}°)")
        print()
        
        print("-" * 60)
        print("Press Ctrl+C to exit")
        
    def run(self, update_rate: float = 2.0):
        """Main monitoring loop."""
        if not self.connect():
            return
            
        last_print = 0
        print_interval = 1.0 / update_rate
        
        try:
            while True:
                msg = self.mav.recv_match(blocking=True, timeout=1.0)
                
                if msg is not None:
                    self.process_message(msg)
                    
                now = time.time()
                if now - last_print >= print_interval:
                    self.print_status()
                    last_print = now
                    
        except KeyboardInterrupt:
            print("\nMonitor stopped.")


def main():
    if not PYMAVLINK_AVAILABLE:
        sys.exit(1)
        
    parser = argparse.ArgumentParser(description='EKF3 Status Monitor')
    
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/ttyACM0',
        help='Serial port or UDP connection (e.g., udp:127.0.0.1:14550)',
    )
    parser.add_argument(
        '--baudrate',
        type=int,
        default=921600,
        help='Serial baud rate',
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=2.0,
        help='Display update rate (Hz)',
    )
    
    args = parser.parse_args()
    
    monitor = EKFMonitor(args.port, args.baudrate)
    monitor.run(args.rate)


if __name__ == '__main__':
    main()
