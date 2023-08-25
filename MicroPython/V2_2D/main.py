import pyb
import machine
import pca9685
import pca9685_servo
import as5048a
import utime
from micropython import const
import math


# Configuration
i2c = machine.SoftI2C("X9","X10",freq=100000)
pca = pca9685.PCA9685(i2c)
servos = pca9685_servo.Servos(pca)
as5048_cs = pyb.Pin("X5", pyb.Pin.OUT_PP)
as5048_cs.high()
as5048_spi = pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER, prescaler=8, bits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs)  # readings from 70-110

# Constants
LL_x = 2
LL_y = 3
RL_x = 0
RL_y = 1
FOLDER = 4
FLAPPER = 5
ATTACK = 6
folded = 160
extended = 50

# Parameters
leg_y = 90
leg_y_amplitude = 0
leg_x = 40  # 50-140
leg_x_amplitude = 0
ellipse_angle = 0.5
attack_angle = 120
flapper = 110

uart = pyb.UART(3, 115200)

# Motors initialization
pca.duty(FLAPPER, 200)
servos.position(ATTACK, attack_angle)
servos.position(FOLDER, extended)
utime.sleep(3)

while True:
    response = uart.read()  # check what happens if several commands are sent
    if response:
        print(response)
        data_str = response.decode('utf-8')
        numbers = data_str.split(',')
        motor, attack_angle = [float(num) for num in numbers]
        pca.duty(FLAPPER, motor)
        servos.position(ATTACK, attack_angle)
        
        start_time = utime.ticks_ms()
        duration = 15000  # 10 seconds in deciseconds

        while utime.ticks_diff(utime.ticks_ms(), start_time) < duration:
            utime.sleep(0.001)

            servos.position(FOLDER, extended)

            y_theta = leg_y
            LL_y_angle = y_theta
            RL_y_angle = 180 - y_theta

            x_theta = leg_x
            RL_x_angle = x_theta
            LL_x_angle = 180 - RL_x_angle

            servos.position(LL_y, LL_y_angle)
            servos.position(RL_y, RL_y_angle)

            servos.position(RL_x, RL_x_angle)
            servos.position(LL_x, LL_x_angle)

        uart.read()  # in case data was sent while in the loop it is deleted
        pca.duty(FLAPPER, 200)

