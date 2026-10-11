#!/usr/bin/env python3
"""
Test 02: ListServos
Scans the bus for all connected servos.
"""

import os
import sys
from st3215 import ST3215

def main():
    print("=== ST3215 List Servos Test ===")
    
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
        
        # Scan for all servos on the bus
        print("Scanning bus for connected servos...")
        
        servo_list = servo.ListServos()
        
        if servo_list is None:
            print("❌ Error occurred during servo scan")
            sys.exit(1)
        elif len(servo_list) == 0:
            print("❌ No servos found on the bus")
            print("   Check connections and power supply")
            sys.exit(1)
        else:
            print(f"✓ Found {len(servo_list)} servo(s):")
            for servo_id in servo_list:
                print(f"   - Servo ID: {servo_id}")
            
            if 1 in servo_list:
                print("✓ Required servo ID 1 is present")
                print("Test completed successfully!")
            else:
                print("⚠️  Warning: Servo ID 1 not found (required for other tests)")
                print("   Make sure servo ID 1 is connected and powered")
            
    except Exception as e:
        print(f"❌ Error during test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()