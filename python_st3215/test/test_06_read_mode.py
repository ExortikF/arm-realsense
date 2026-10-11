#!/usr/bin/env python3
"""
Test 06: ReadMode
Tests operational mode reading.
"""

import os
import sys
from st3215 import ST3215

def main():
    print("=== ST3215 Mode Test ===")
    
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
        
        # Test ReadMode
        print(f"Reading operational mode from servo ID {servo_id}...")
        
        mode = servo.ReadMode(servo_id)
        
        if mode is not None:
            print(f"✓ Current Mode: {mode}")
            
            # Interpret mode value according to documentation
            mode_descriptions = {
                0: "Position servo mode",
                1: "Motor constant speed mode",
                2: "PWM open-loop speed regulation mode", 
                3: "Step servo mode"
            }
            
            if mode in mode_descriptions:
                print(f"  Description: {mode_descriptions[mode]}")
            else:
                print(f"  Description: Unknown mode ({mode})")
            
            print("Test completed successfully!")
        else:
            print("❌ Failed to read operational mode")
            print("   Check servo connection and power supply")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ Error during test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()