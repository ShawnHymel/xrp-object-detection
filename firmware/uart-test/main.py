from machine import UART, Pin
import time
import json

# Settings
UART_BAUDRATE = 115200

# Configure UART
uart = UART(
    0,
    baudrate=UART_BAUDRATE,
    tx=Pin(0),
    rx=Pin(1),
    timeout=200,
)

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

# Do forever
while True:
    
    # Read bounding box info from UART
    bbox_basket, bbox_target = get_bboxes(uart)
    
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
        
    # Print center coordinates
    print(f"{x_center}, {y_center}")
