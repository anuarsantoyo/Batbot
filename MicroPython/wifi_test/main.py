import pyb
import time
uart = pyb.UART(3, 115200)
uart.write('AT\r\n')

while True:
    response = uart.read()
    if response:
        data_str = response.decode('utf-8')
        numbers = data_str.split(',')
        a, b, c, d = [int(num) for num in numbers]
        print(a+b)
        time.sleep(3)
        uart.read()
