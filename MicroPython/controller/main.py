import pyb
import machine
import pca9685
import pca9685_servo
import as5048a
import utime
from micropython import const
import math
import time

# Configuration
i2c = machine.SoftI2C("X9","X10",freq=100000)  # For PCA
pca = pca9685.PCA9685(i2c)
servos = pca9685_servo.Servos(pca)
as5048_cs = pyb.Pin("X5", pyb.Pin.OUT_PP)  # For magnetic encoder (wing angle measurement)
as5048_cs.high()
as5048_spi = pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER, prescaler=8, bits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs)  # readings from 235-194 -> down-up
uart = pyb.UART(3, 115200)  # Wireless uart to send commands.

# Constants
RL_x = 0
RL_y = 1
LL_x = 2
LL_y = 3
FOLDER = 4
FLAPPER = 5
#ATTACK = 6
folded = 160
extended = 50

angle_max = 152.688
angle_min = 119.218

# Motors initialization
pca.duty(FLAPPER, 200)
#servos.position(ATTACK, 120)
servos.position(FOLDER, extended)
utime.sleep(3)

while True:
    response = uart.read()  # read command
    if response:
        print(response)
        data_str = response.decode('utf-8')
        numbers = data_str.split(',')
        x, y, z = [float(num) for num in numbers]  # Extract command

        y_theta = y  # Calculate angle from body plane to leg in vertical
        if y_theta < 30:  # Stops angle from exceeding limit values (also allowing straight trajectories in leg)
            y_theta = 30
        elif y_theta > 150:
            y_theta = 150
        LL_y_angle = y_theta
        RL_y_angle = 180 - y_theta  # Invert for right leg

        # Calculate angle from body plane to leg in vertical
        x_theta = x
        if x_theta < 40:
            x_theta = 40
        elif x_theta > 120:
            x_theta = 120

        RL_x_angle = x_theta
        LL_x_angle = 180 - RL_x_angle

        # Command servos
        servos.position(LL_y, LL_y_angle)
        servos.position(RL_y, RL_y_angle)

        servos.position(RL_x, RL_x_angle)
        servos.position(LL_x, LL_x_angle)

        #servos.position(ATTACK, attack_angle)
        pca.duty(FLAPPER, z)

        uart.read()  # in case data was sent while in the loop it is deleted
        #pca.duty(FLAPPER, 200)  # Turn off flapper

