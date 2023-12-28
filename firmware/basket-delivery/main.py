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

# Do forever
while True:
    
    # Wait for UART data
    line = uart.readline()
    if line is None:
        continue

    # Parse JSON string
    try:
        line = json.loads(line.decode("utf-8"))
    except Exception as e:
        print(f"ERROR: {e}")
        continue

    print(line)