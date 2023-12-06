# Based on https://github.com/micropython/micropython/blob/master/examples/rp2/pio_uart_rx.py

import time
from collections import deque

from machine import Pin, UART
from rp2 import PIO, StateMachine, asm_pio

# Settings
UART_BAUD = 115200
PIO_RX_PIN = Pin(27, Pin.IN, Pin.PULL_UP)
PIO_INDEX = 5
QUEUE_MAX = 500

# Global queue
rx_deque = deque((), QUEUE_MAX)

# PIO: UART receiver
@asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_RIGHT,
)
def uart_rx():
    label("start")
    wait(0, pin, 0)
    set(x, 7)                 [10]
    label("bitloop")
    in_(pins, 1)
    jmp(x_dec, "bitloop")     [6]
    jmp(pin, "good_stop")
    wait(1, pin, 0)
    jmp("start")
    label("good_stop")
    push(block)
    #irq(block, rel(0))

def put_rx_char(sm):
    rx_deque.append(chr(sm.get() >> 24))

# Test this module
def main():
    
    led = Pin("LED", Pin.OUT)
    
    # Blink 1 times (time to hit "stop")
    for i in range(1):
        led.on()
        time.sleep(0.5)
        led.off()
        time.sleep(0.5)
    
    # Set up the state machine we're going to use to receive the characters.
    sm = StateMachine(
        PIO_INDEX,
        uart_rx,
        freq=8 * UART_BAUD,
        in_base=PIO_RX_PIN,  # For WAIT, IN
        jmp_pin=PIO_RX_PIN,  # For JMP
    )
    sm.irq(put_rx_char)
    sm.active(1)
    
    # Print whatever we receive from the UART
    print("Receiving from UART in batches...")
    while True:
        print(chr(sm.get() >> 24), end="")
        #while len(rx_deque) > 0:
        #    print(rx_deque.popleft(), end="")

if __name__ == "__main__":
    main()

