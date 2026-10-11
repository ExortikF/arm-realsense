#!/usr/bin/env python3
"""
Test 05: ReadAcceleration
Tests acceleration parameter reading.
"""

import os
import sys
from st3215 import ST3215

def main():
    print("=== ST3215 Acceleration Test ===")
    
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
        
        servo_id = 1
        
        # Test ReadAcceleration
        print(f"Reading acceleration setting from servo ID {servo_id}...")
        
        acceleration = servo.ReadAccelaration(servo_id)  # Note: method name has typo in original API
        
        if acceleration is not None:
            print(f"✓ Current Acceleration: {acceleration} step/s²")
            
            # Convert to more readable units (from documentation: unit is 100 step/s²)
            actual_acceleration = acceleration * 100
            print(f"  Actual acceleration: {actual_acceleration} step/s²")
            
            print("Test completed successfully!")
        else:
            print("❌ Failed to read acceleration setting")
            print("   Check servo connection and power supply")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ Error during test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()