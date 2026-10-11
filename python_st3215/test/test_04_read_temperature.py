#!/usr/bin/env python3
"""
Test 04: ReadTemperature
Tests temperature sensor reading.
"""

import os
import sys
from st3215 import ST3215

def main():
    print("=== ST3215 Temperature Test ===")
    
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
        
        # Test ReadTemperature
        print(f"Reading internal temperature from servo ID {servo_id}...")
        
        temperature = servo.ReadTemperature(servo_id)
        
        if temperature is not None:
            print(f"✓ Internal Temperature: {temperature}°C")
            
            # Provide some context about the temperature reading
            if temperature < 30:
                print("  Status: Normal (cool)")
            elif temperature < 50:
                print("  Status: Normal (warm)")
            elif temperature < 65:
                print("  Status: Getting hot")
            else:
                print("  Status: ⚠️ Hot - monitor closely")
            
            print("Test completed successfully!")
        else:
            print("❌ Failed to read internal temperature")
            print("   Check servo connection and power supply")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ Error during test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()