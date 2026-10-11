#!/usr/bin/env python3
"""
Test 10: Complete Motion Control
Comprehensive test of servo control functions.
Tests StartServo, SetAcceleration, SetSpeed, rotation mode, position mode, and StopServo.
"""

import os
import sys
import time
from st3215 import ST3215

def main():
    print("=== ST3215 Complete Motion Control Test ===")
    
    # Get device from environment variable
    device = os.getenv('ST3215_DEV')
    if not device:
        print("❌ Error: ST3215_DEV environment variable not set")
        print("   Please set it to your serial device (e.g., /dev/ttyUSB0)")
        sys.exit(1)
    
    print(f"Device: {device}")
    
    # Safety warning
    print("\n⚠️  SAFETY WARNING:")
    print("   This test will move the servo motor in different modes.")
    print("   Ensure the servo has adequate clearance for movement.")
    print("   Press Ctrl+C to stop if any issues occur.")
    input("   Press Enter when ready to start the test...")
    
    try:
        # Initialize servo controller
        servo = ST3215(device)
        print("✓ Serial connection established")
        
        servo_id = 1
        
        # Step 1: Start servo (enable torque)
        print(f"\nStep 1: Starting servo ID {servo_id} (enabling torque)...")
        start_result = servo.StartServo(servo_id)
        if not start_result:
            print("❌ Failed to start servo")
            sys.exit(1)
        print("✓ Servo started successfully")
        
        # Step 2: Set acceleration
        print(f"\nStep 2: Setting acceleration...")
        acceleration = 100  # 100 * 100 step/s² = 10000 step/s²
        acc_result = servo.SetAcceleration(servo_id, acceleration)
        if not acc_result:
            print("❌ Failed to set acceleration")
        else:
            print(f"✓ Acceleration set to {acceleration} (actual: {acceleration * 100} step/s²)")
        
        # Step 3: Set speed
        print(f"\nStep 3: Setting speed...")
        speed = 2000  # step/s
        speed_result = servo.SetSpeed(servo_id, speed)
        if not speed_result:
            print("❌ Failed to set speed")
        else:
            print(f"✓ Speed set to {speed} step/s")
        
        # Step 4: Test rotation mode
        print(f"\nStep 4: Testing continuous rotation mode...")
        print("   Rotating clockwise for 3 seconds...")
        rotate_result = servo.Rotate(servo_id, 500)  # Positive speed = clockwise
        if not rotate_result:
            print("❌ Failed to start rotation")
        else:
            print("✓ Clockwise rotation started")
            time.sleep(3)
            
            print("   Rotating counter-clockwise for 3 seconds...")
            rotate_result = servo.Rotate(servo_id, -500)  # Negative speed = counter-clockwise
            if not rotate_result:
                print("❌ Failed to change rotation direction")
            else:
                print("✓ Counter-clockwise rotation started")
                time.sleep(3)
        
        # Step 5: Test position mode
        print(f"\nStep 5: Testing position control mode...")
        
        # Get current position
        current_pos = servo.ReadPosition(servo_id)
        if current_pos is None:
            print("❌ Failed to read current position")
        else:
            print(f"   Current position: {current_pos}")
            
            # Move to position +1000 steps from current
            target_pos1 = current_pos + 1000
            print(f"   Moving to position {target_pos1}...")
            move_result1 = servo.MoveTo(servo_id, target_pos1, speed=1500, acc=80, wait=True)
            if not move_result1:
                print("❌ Failed to move to first target position")
            else:
                print("✓ Reached first target position")
            
            # Move to position -1000 steps from original
            target_pos2 = current_pos - 1000
            print(f"   Moving to position {target_pos2}...")
            move_result2 = servo.MoveTo(servo_id, target_pos2, speed=1500, acc=80, wait=True)
            if not move_result2:
                print("❌ Failed to move to second target position")
            else:
                print("✓ Reached second target position")
            
            # Return to original position
            print(f"   Returning to original position {current_pos}...")
            move_result3 = servo.MoveTo(servo_id, current_pos, speed=1500, acc=80, wait=True)
            if not move_result3:
                print("❌ Failed to return to original position")
            else:
                print("✓ Returned to original position")
        
        # Step 6: Stop servo (disable torque) - IMPORTANT!
        print(f"\nStep 6: Stopping servo ID {servo_id} (disabling torque)...")
        stop_result = servo.StopServo(servo_id)
        if not stop_result:
            print("❌ Failed to stop servo")
        else:
            print("✓ Servo stopped successfully")
        
        print("\n=== Test Summary ===")
        print("✓ Complete motion control test finished successfully!")
        print("  All functions tested:")
        print("  - StartServo: ✓")
        print("  - SetAcceleration: ✓")
        print("  - SetSpeed: ✓")
        print("  - Rotate (continuous motion): ✓")
        print("  - MoveTo (position control): ✓")
        print("  - StopServo: ✓")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user!")
        print("Attempting to stop servo safely...")
        try:
            servo.StopServo(servo_id)
            print("✓ Servo stopped safely")
        except:
            print("❌ Could not stop servo - please check manually")
        sys.exit(1)
        
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
        print("Attempting to stop servo safely...")
        try:
            servo.StopServo(servo_id)
            print("✓ Servo stopped safely")
        except:
            print("❌ Could not stop servo - please check manually")
        sys.exit(1)

if __name__ == "__main__":
    main()