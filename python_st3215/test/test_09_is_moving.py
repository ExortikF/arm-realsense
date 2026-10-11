#!/usr/bin/env python3
"""
Test 09: IsMoving
Tests motion detection.
"""

import os
import sys
import time
from st3215 import ST3215

def main():
    print("=== ST3215 Motion Detection Test ===")
    
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
        
        # Test IsMoving when stationary
        print(f"Testing motion detection on servo ID {servo_id}...")
        print("Step 1: Checking if servo is currently moving (should be stationary)...")
        
        is_moving_initial = servo.IsMoving(servo_id)
        
        if is_moving_initial is None:
            print("❌ Failed to read motion status")
            print("   Check servo connection and power supply")
            sys.exit(1)
        
        print(f"✓ Initial motion status: {'Moving' if is_moving_initial else 'Stationary'}")
        
        # Start servo and initiate movement to test motion detection
        print("\nStep 2: Starting servo and initiating movement...")
        
        # Enable servo
        start_result = servo.StartServo(servo_id)
        if not start_result:
            print("❌ Failed to start servo")
            sys.exit(1)
        
        print("✓ Servo enabled")
        
        # Get current position and move to a different position
        current_pos = servo.ReadPosition(servo_id)
        if current_pos is None:
            print("❌ Failed to read current position")
            sys.exit(1)
        
        # Calculate target position (move 500 steps from current position)
        target_pos = current_pos + 500
        print(f"Moving from position {current_pos} to {target_pos}...")
        
        # Start movement
        move_result = servo.MoveTo(servo_id, target_pos, speed=1000, acc=50, wait=False)
        if not move_result:
            print("❌ Failed to initiate movement")
            sys.exit(1)
        
        # Check motion status during movement
        print("Step 3: Checking motion status during movement...")
        time.sleep(0.1)  # Small delay to ensure movement has started
        
        is_moving_during = servo.IsMoving(servo_id)
        if is_moving_during is None:
            print("❌ Failed to read motion status during movement")
        else:
            print(f"✓ Motion status during movement: {'Moving' if is_moving_during else 'Stationary'}")
        
        # Wait for movement to complete and check again
        print("Step 4: Waiting for movement to complete...")
        time.sleep(2)  # Wait for movement to finish
        
        is_moving_final = servo.IsMoving(servo_id)
        if is_moving_final is None:
            print("❌ Failed to read final motion status")
        else:
            print(f"✓ Final motion status: {'Moving' if is_moving_final else 'Stationary'}")
        
        # Stop servo
        servo.StopServo(servo_id)
        print("✓ Servo disabled")
        
        print("Test completed successfully!")
        
    except Exception as e:
        print(f"❌ Error during test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()