import pyb
import machine
import pca9685
import pca9685_servo
import as5048a
import time
import math

i2c = machine.SoftI2C("X9", "X10", freq=100000)
pca = pca9685.PCA9685(i2c)
servos = pca9685_servo.Servos(pca)
as5048_cs = pyb.Pin("X5", pyb.Pin.OUT_PP)
as5048_cs.high()
as5048_spi=pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER,prescaler=8,bits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs)

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
leg_y = 30
leg_x = 140  # 50-140


pca.duty(FLAPPER,200)
    
y_theta = leg_y
LL_y_angle = y_theta
RL_y_angle =  180 - y_theta
    
x_theta = leg_x
RL_x_angle =  x_theta
LL_x_angle = 180 - RL_x_angle

servos.position(LL_y, LL_y_angle)
servos.position(RL_y, RL_y_angle)

servos.position(RL_x, RL_x_angle)
servos.position(LL_x, LL_x_angle)
