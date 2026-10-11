#!/usr/bin/env python3
"""
Test 01: PingServo
Tests basic communication with servo ID 1.
"""

import os
import sys
from st3215 import ST3215

def main():
    print("=== ST3215 Ping Test ===")
    
    # Get device from environment variable
    device = os.getenv('ST3215_DEV')
    if not device:
        print("❌ Error: ST3215_DEV environment variable not set")
        print("   Please set it to your serial device (e.g., /dev/ttyUSB0)")
        sys.exit(1)
    
    print(f"Device: {device}")
    
    try:
        # Initialize servo controller
        servo = ST3215(device)
        print("✓ Serial connection established")
        
        # Test ping to servo ID 1
        servo_id = 1
        print(f"Testing ping to servo ID {servo_id}...")
        
        result = servo.PingServo(servo_id)
        
        if result:
            print(f"✓ Servo ID {servo_id} responded successfully")
            print("Test completed successfully!")
        else:
            print(f"❌ Servo ID {servo_id} did not respond")
            print("   Check connections, power supply, and servo ID")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ Error during test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()