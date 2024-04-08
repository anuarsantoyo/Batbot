import pyb
import time

uart = pyb.UART(3, 115200)  # Wireless uart to send commands.
while True:
    response = uart.read()  # read comman
    print(response)
    time.sleep(0.1)



