"""
XRP Basket Delivery

This program receives bounding box information over UART from a separate
platform that performs object detection. It was tested with the Google Coral
Micro board trained to identify a red 3D printed basket and a target area
marked with blue tape on the ground. The goal is to pick up the basket and
deliver it to the target area.

Tested with MicroPython v1.12.0 and XRPLib v1.0.1.

Author: Shawn Hymel
Date: December 28, 2023
License: https://opensource.org/license/0bsd/
"""

import time
import json
import sys

from machine import UART, Pin

from XRPLib.differential_drive import DifferentialDrive
from XRPLib.servo import Servo

# ------------------------------------------------------------------------------
# Globals

# Settings
SERVO_PORT = 1
UART_BAUDRATE = 115200
RESTART_WAIT = 5.0              # How long to wait for Coral Micro to boot (sec)
SEARCH_TURN_SPEED = 20.0        # Drive speed when searching
LINEUP_TURN_SPEED = 12.0        # Drive speed when lining up object
DRIVE_SPEED = 20.0              # Drive speed when not bound by distance
MAX_EFFORT = 0.5                # Drive speed when driving by distance
BASKET_X_TARGET = 0.4           # Where the basket X should be for pickup
BASKET_X_DEADZONE = 0.02        # Basket center X can be lined up +/- this val
BASKET_Y_TARGET = 0.82          # Where the basket Y should be for pickup
BASKET_Y_DEADZONE = 0.02        # Basket center Y can be lined up +/- this val
TARGET_X_TARGET = 0.36          # Where the target X should be for dropoff
TARGET_X_DEADZONE = 0.02        # Target center X can be lined up +/- this val
TARGET_Y_TARGET = 0.8           # Where the target Y should be for dropoff
TARGET_Y_DEADZONE = 0.05        # Target center Y can be lined up +/- this val
SERVO_HOME = 180.0              # Arm in the collapsed position (degrees)
SERVO_PICKUP = 12.0             # Arm in the pickup position (degrees)
SERVO_CARRY = 40.0              # Arm in the carry basket position (degrees)
PICKUP_INCREMENTS = 10          # Perform servo lift in increments
PICKUP_DISTANCE = 18.0          # How far to drive backwards to get basket (cm)
DROPOFF_DISTANCE = 15.0         # How far to drive backwards to drop off (cm)
VICTORY_TURN_DEGREES = 90.0     # How far to dance
NUM_VICTORY_TURNS = 2           # How many back and forth turns to do

# Magical empirical constant to perform degrees of robot rotation using
# encoder raw output (warning: not accurate)
WHEEL_ROT_PER_ROBOT_ROT = 2.42

# Configure UART
uart = UART(
    0,
    baudrate=UART_BAUDRATE,
    tx=Pin(0),
    rx=Pin(1),
    timeout=200,
)

# Configure encoded motors and servo
drivetrain = DifferentialDrive.get_default_differential_drive()
servo = Servo.get_default_servo(SERVO_PORT)

# State machine to control robot's behaviors
# 0: Look for the basket
# 1: Drive toward the basket
# 2: Pick up the basket
# 3: Look for the target
# 4: Drive toward the target
# 5: Deliver basket to the target
# 6: Victory dance
# 7: Do nothing
current_state = 0
    
# ------------------------------------------------------------------------------
# Functions

def get_bboxes(uart):
    """Get object detection bounding boxes from UART"""
    
    # Wait for UART data
    line = uart.readline()
    if line is None:
        return (None, None)

    # Parse JSON string
    try:
        bboxes = json.loads(line.decode("utf-8"))
    except Exception as e:
        print(f"ERROR: {e}")
        return (None, None)
    
    # Find max score of each bounding box class
    bbox_basket = None
    bbox_target = None
    for bbox in bboxes["bboxes"]:
        if bbox["id"] == 1:
            if bbox_basket == None:
                bbox_basket = bbox
            else:
                if bbox["score"] > bbox_basket["score"]:
                    bbox_basket = bbox
        if bbox["id"] == 2:
            if bbox_target == None:
                bbox_target = bbox
            else:
                if bbox["score"] > bbox_target["score"]:
                    bbox_target = bbox
            
    
    return (bbox_basket, bbox_target)

def turn(drivetrain, degrees):
    """
    Very hacky way of turning: simply measure one encoder until it reaches a set
    point. This is because drivetrain.turn() is bugged and enter a forever loop
    of turning. Warning: this function is not very accurate!
    """
    
    # Get starting positions
    left_start = drivetrain.left_motor.get_position()
    right_start = drivetrain.right_motor.get_position()
    
    # Set direction and speed
    if degrees == 0.0:
        return
    elif degrees > 0.0:
        drivetrain.set_speed(
            left_speed=SEARCH_TURN_SPEED,
            right_speed=-SEARCH_TURN_SPEED
        )
        target_pos = (degrees / 360) * WHEEL_ROT_PER_ROBOT_ROT
    elif degrees < 0.0:
        drivetrain.set_speed(
            left_speed=-SEARCH_TURN_SPEED,
            right_speed=SEARCH_TURN_SPEED
        )
        target_pos = -(degrees / 360) * WHEEL_ROT_PER_ROBOT_ROT
        
    # Turn until desired position
    while True:
        left_pos = drivetrain.left_motor.get_position()
        right_pos = drivetrain.right_motor.get_position()
        if (abs(left_pos - left_start) >= target_pos) or \
            (abs(right_pos - right_start) >= target_pos):
            break

    drivetrain.set_speed(left_speed=0.0, right_speed=0.0)

# ------------------------------------------------------------------------------
# Main

def main():
    
    global uart
    global drivetrain
    global servo
    global current_state
    
    # Setup
    servo.set_angle(SERVO_HOME)
    drivetrain.set_speed(
        left_speed=0.0,
        right_speed=0.0,
    )
    wait_counter = 0

    # Wait for Coral Micro to start running
    print("Waiting for Coral Micro to boot...")
    time.sleep(RESTART_WAIT)
    print("Go!")

    # Loop
    while True:
        
        # Always receive bounding box info on each iteration
        bbox_basket, bbox_target = get_bboxes(uart)
        
        # State 0: look for the basket
        if current_state == 0:
            
            # See if there is a basket in view
            if bbox_basket:
                print("Basket found")
                drivetrain.set_speed(
                    left_speed=0.0,
                    right_speed=0.0,
                )
                current_state = 1
                continue
            
            # If not, rotate the robot
            drivetrain.set_speed(
                left_speed=SEARCH_TURN_SPEED,
                right_speed=-SEARCH_TURN_SPEED,
            )
            
        # State 1: drive toward the basket
        elif current_state == 1:
            
            # Find basket bounding box
            if bbox_basket is None:
                print("Basket lost. Searching...")
                current_state = 0
                continue
            
            # Get coordinates of basket
            width = bbox_basket["xmax"] - bbox_basket["xmin"]
            height = bbox_basket["ymax"] - bbox_basket["ymin"]
            x_center = bbox_basket["xmin"] + (width / 2.0)
            y_center = bbox_basket["ymin"] + (height / 2.0)
            
            # Print debug string
            print(f"Lining up basket: ({x_center:.3f}, {y_center:.3f})")
            
            # Line up basket in the horizontal center of the frame
            x_done = False
            if x_center < (BASKET_X_TARGET - BASKET_X_DEADZONE):
                drivetrain.set_speed(
                    left_speed=-LINEUP_TURN_SPEED,
                    right_speed=LINEUP_TURN_SPEED,
                )
            elif x_center > (BASKET_X_TARGET + BASKET_X_DEADZONE):
                drivetrain.set_speed(
                    left_speed=LINEUP_TURN_SPEED,
                    right_speed=-LINEUP_TURN_SPEED,
                )
            else:
                drivetrain.set_speed(
                    left_speed=0.0,
                    right_speed=0.0
                )
                x_done = True
                
            # Keep lining up x if needed
            if not x_done:
                continue
                
            # If X is lined up, line up Y
            if y_center < (BASKET_Y_TARGET - BASKET_Y_DEADZONE):
                drivetrain.set_speed(
                    left_speed=DRIVE_SPEED,
                    right_speed=DRIVE_SPEED,
                )
                wait_counter = 0
            elif y_center > (BASKET_Y_TARGET + BASKET_Y_DEADZONE):
                drivetrain.set_speed(
                    left_speed=-DRIVE_SPEED,
                    right_speed=-DRIVE_SPEED,
                )
                wait_counter = 0
            else:
                
                # Make sure basket is lined up 5 times in a row
                wait_counter += 1
                if wait_counter >= 5:
                    print(f"Getting basket: ({x_center:.3f}, {y_center:.3f})")
                    drivetrain.set_speed(
                        left_speed=0.0,
                        right_speed=0.0,
                    )
                    time.sleep(1.0)
                    current_state = 2
                    continue
            
        # State 2: pick up basket
        elif current_state == 2:
            
            # Deploy arm
            servo.set_angle(SERVO_PICKUP)
            
            # Drive forward to pick up basket
            drivetrain.straight(
                distance=PICKUP_DISTANCE,
                max_effort=MAX_EFFORT,
            )
            
            # Pick up basket (somewhat slowly)
            period = 1 / PICKUP_INCREMENTS
            for i in range(PICKUP_INCREMENTS):
                servo.set_angle(
                    ((SERVO_CARRY - SERVO_PICKUP) * (period * i)) + SERVO_PICKUP
                )
                time.sleep(period)
            servo.set_angle(SERVO_CARRY)
            time.sleep(1.0)
            
            # Continue to state 3
            print("Searching for target...")
            current_state = 3
            continue
            
        # State 3: look for the target
        elif current_state == 3:
            
            # See if there is a target zone in view
            if bbox_target:
                print("Target found")
                drivetrain.set_speed(
                    left_speed=0.0,
                    right_speed=0.0,
                )
                current_state = 4
                continue
            
            # If not, rotate the robot
            drivetrain.set_speed(
                left_speed=SEARCH_TURN_SPEED,
                right_speed=-SEARCH_TURN_SPEED,
            )
            
        # State 4: drive toward the target
        elif current_state == 4:
            
            # Find basket bounding box
            if bbox_target is None:
                print("Target lost. Searching...")
                current_state = 3
                continue
            
            # Get coordinates of target
            width = bbox_target["xmax"] - bbox_target["xmin"]
            height = bbox_target["ymax"] - bbox_target["ymin"]
            x_center = bbox_target["xmin"] + (width / 2.0)
            y_center = bbox_target["ymin"] + (height / 2.0)
            
            # Print debug string
            print(f"Lining up target: ({x_center:.3f}, {y_center:.3f})")
            
            # Line up target in the horizontal center of the frame
            x_done = False
            if x_center < (TARGET_X_TARGET - TARGET_X_DEADZONE):
                drivetrain.set_speed(
                    left_speed=-LINEUP_TURN_SPEED,
                    right_speed=LINEUP_TURN_SPEED,
                )
            elif x_center > (TARGET_X_TARGET + TARGET_X_DEADZONE):
                drivetrain.set_speed(
                    left_speed=LINEUP_TURN_SPEED,
                    right_speed=-LINEUP_TURN_SPEED,
                )
            else:
                drivetrain.set_speed(
                    left_speed=0.0,
                    right_speed=0.0
                )
                x_done = True
                
            # Keep lining up x if needed
            if not x_done:
                continue
                
            # If X is lined up, line up Y
            if y_center < (TARGET_Y_TARGET - TARGET_Y_DEADZONE):
                drivetrain.set_speed(
                    left_speed=DRIVE_SPEED,
                    right_speed=DRIVE_SPEED,
                )
                wait_counter = 0
            elif y_center > (TARGET_Y_TARGET + TARGET_Y_DEADZONE):
                drivetrain.set_speed(
                    left_speed=-DRIVE_SPEED,
                    right_speed=-DRIVE_SPEED,
                )
                wait_counter = 0
                
            else:
                
                # Make sure basket is lined up 5 times in a row
                wait_counter += 1
                if wait_counter >= 5:
                    print("Target is lined up")
                    drivetrain.set_speed(
                        left_speed=0.0,
                        right_speed=0.0,
                    )
                    current_state = 5
                    continue
            
        # State 5: drop off basket
        elif current_state == 5:
            
            # Drive forward to dropoff zone
            drivetrain.straight(
                distance=DROPOFF_DISTANCE,
                max_effort=MAX_EFFORT,
            )
            
            # Drop off basket
            servo.set_angle(SERVO_PICKUP)
            time.sleep(1.0)
            
            # Back away from dropoff zone
            drivetrain.straight(
                distance=DROPOFF_DISTANCE,
                max_effort=-MAX_EFFORT,
            )
            
            # Retract arm
            servo.set_angle(SERVO_HOME)
            
            # Continue to state 6
            print("Dropoff complete")
            current_state = 6
            continue
        
        # State 6: victory dance!
        elif current_state == 6:
            for i in range(NUM_VICTORY_TURNS):
                turn(drivetrain, VICTORY_TURN_DEGREES)
                time.sleep(0.2)
                turn(drivetrain, -VICTORY_TURN_DEGREES)
                time.sleep(0.2)
            current_state = 7
            continue
        
        # State 7: all done, do nothing
        else:
            while True:
                print("All done, please reset me")
                time.sleep(5.0)
                
# Run main and kill motors on exit
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program done. Stopping...")
        drivetrain.set_speed(left_speed=0.0, right_speed=0.0)
        sys.exit(0)