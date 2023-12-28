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

from machine import UART, Pin

from XRPLib.differential_drive import DifferentialDrive
from XRPLib.servo import Servo

# ------------------------------------------------------------------------------
# Globals

# Settings
SERVO_PORT = 1
UART_BAUDRATE = 115200
TURN_SPEED = 15.0               # Drive speed when turning/searching
DRIVE_SPEED = 20.0              # Drive speed when not bound by distance
MAX_EFFORT = 0.4                # Drive speed when driving by distance
BASKET_X_DEADZONE = 0.03        # Basket center X can be lined up 0.5 +/- 0.03
BASKET_Y_TARGET = 0.8           # Basket center Y should be 0.8
BASKET_Y_DEADZONE = 0.05        # Basket center Y can be lined up 0.8 +/- 0.05
SERVO_HOME = 180.0              # Arm in the collapsed position (degrees)
SERVO_PICKUP = 12.0             # Arm in the pickup position (degrees)
SERVO_CARRY = 40.0              # Arm in the carry basket position (degrees)
PICKUP_TURN_DEGREES = 180.0     # How many degrees to turn to pick up basket
PICKUP_DISTANCE = 18.0          # How far to drive backwards to get basket (cm)

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

# ------------------------------------------------------------------------------
# Main

# Setup
servo.set_angle(SERVO_HOME)
drivetrain.set_speed(
    left_speed=0.0,
    right_speed=0.0,
)
time.sleep(2.0)

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
            left_speed=TURN_SPEED,
            right_speed=-TURN_SPEED,
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
        print(f"Lining up: ({x_center:.3f}, {y_center:.3f})")
        
        # Line up basket in the horizontal center of the frame
        x_done = False
        if x_center < (0.5 - BASKET_X_DEADZONE):
            drivetrain.set_speed(
                left_speed=-TURN_SPEED,
                right_speed=TURN_SPEED,
            )
        elif x_center > (0.5 + BASKET_X_DEADZONE):
            drivetrain.set_speed(
                left_speed=TURN_SPEED,
                right_speed=-TURN_SPEED,
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
        elif y_center > (BASKET_Y_TARGET + BASKET_Y_DEADZONE):
            drivetrain.set_speed(
                left_speed=-DRIVE_SPEED,
                right_speed=-DRIVE_SPEED,
            )
        else:
            print("Basket is lined up")
            drivetrain.set_speed(
                left_speed=0.0,
                right_speed=0.0,
            )
            current_state = 2
            continue
        
    # State 2: pick up basket
    elif current_state == 2:
        
        # Turn around 180 degrees
        drivetrain.turn(
            turn_degrees=PICKUP_TURN_DEGREES,
            max_effort=MAX_EFFORT,
        )
        
        # Deploy arm
        servo.set_angle(SERVO_PICKUP)
        
        # Drive backwards to pick up basket
        drivetrain.straight(
            distance=PICKUP_DISTANCE,
            max_effort=-MAX_EFFORT,
        )
        
        # Pick up basket
        servo.set_angle(SERVO_CARRY)
        time.sleep(1.0)
        
        # Turn back around
        drivetrain.turn(
            turn_degrees=-PICKUP_TURN_DEGREES,
            max_effort=MAX_EFFORT,
        )
        
        while True:
            print("done")
            time.sleep(1.0)