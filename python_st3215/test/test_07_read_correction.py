#!/usr/bin/env python3
"""
Test 07: ReadCorrection
Tests position correction reading.
"""

import os
import sys
from st3215 import ST3215

def main():
    print("=== ST3215 Position Correction Test ===")
    
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
        
        # Test ReadCorrection
        print(f"Reading position correction from servo ID {servo_id}...")
        
        correction = servo.ReadCorrection(servo_id)
        
        if correction is not None:
            print(f"✓ Position Correction: {correction} steps")
            
            # Provide context about the correction value
            if correction == 0:
                print("  Status: No position correction applied")
            elif correction > 0:
                print(f"  Status: Positive correction of {correction} steps")
            else:
                print(f"  Status: Negative correction of {abs(correction)} steps")
            
            print("  Range: -2047 to +2047 steps")
            print("Test completed successfully!")
        else:
            print("❌ Failed to read position correction")
            print("   Check servo connection and power supply")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ Error during test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()