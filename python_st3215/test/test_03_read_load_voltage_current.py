#!/usr/bin/env python3
"""
Test 03: ReadLoad + ReadVoltage + ReadCurrent
Tests telemetry reading functions.
Special Instructions: Apply physical force to servo shaft before running.
"""

import os
import sys
from st3215 import ST3215

def main():
    print("=== ST3215 Load, Voltage & Current Test ===")
    
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
        
        # Special instructions for this test
        print("\n⚠️  SPECIAL INSTRUCTIONS:")
        print("   1. Apply physical force to the servo motor shaft")
        print("   2. Hold the force while the test is running")
        print("   3. Press Enter when ready to start the test")
        
        input("Press Enter to continue...")
        print("\nRunning telemetry tests...")
        
        # Test ReadLoad
        print(f"Reading motor load from servo ID {servo_id}...")
        load = servo.ReadLoad(servo_id)
        
        if load is not None:
            print(f"✓ Motor Load: {load:.2f}%")
        else:
            print("❌ Failed to read motor load")
        
        # Test ReadVoltage
        print(f"Reading supply voltage from servo ID {servo_id}...")
        voltage = servo.ReadVoltage(servo_id)
        
        if voltage is not None:
            print(f"✓ Supply Voltage: {voltage:.2f}V")
        else:
            print("❌ Failed to read supply voltage")
        
        # Test ReadCurrent
        print(f"Reading current consumption from servo ID {servo_id}...")
        current = servo.ReadCurrent(servo_id)
        
        if current is not None:
            print(f"✓ Current Consumption: {current:.2f}mA")
        else:
            print("❌ Failed to read current consumption")
        
        # Summary
        print("\n=== Test Summary ===")
        if load is not None and voltage is not None and current is not None:
            print("✓ All telemetry readings successful!")
            print(f"  Load: {load:.2f}%")
            print(f"  Voltage: {voltage:.2f}V") 
            print(f"  Current: {current:.2f}mA")
            print("Test completed successfully!")
        else:
            print("❌ Some telemetry readings failed")
            print("   Check servo connection and power supply")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ Error during test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()