#!/usr/bin/env python3
"""
Test 08: ReadStatus
Tests servo status reading.
"""

import os
import sys
from st3215 import ST3215

def main():
    print("=== ST3215 Status Test ===")
    
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
        
        # Test ReadStatus
        print(f"Reading sensor status from servo ID {servo_id}...")
        
        status = servo.ReadStatus(servo_id)
        
        if status is not None:
            print("✓ Sensor Status:")
            
            # Display each sensor status
            for sensor, state in status.items():
                status_symbol = "✓" if state else "❌"
                print(f"  {sensor}: {status_symbol} {'OK' if state else 'ERROR'}")
            
            # Check if any sensors have errors
            errors = [sensor for sensor, state in status.items() if not state]
            if errors:
                print(f"\n⚠️  Warning: {len(errors)} sensor(s) reporting errors:")
                for sensor in errors:
                    print(f"   - {sensor}")
            else:
                print("\n✓ All sensors reporting OK")
            
            print("Test completed successfully!")
        else:
            print("❌ Failed to read sensor status")
            print("   Check servo connection and power supply")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ Error during test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()