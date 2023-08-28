import pyb
import machine
import pca9685
import pca9685_servo
import as5048a
import time
import math
i2c = machine.SoftI2C("X9","X10",freq=100000)
pca = pca9685.PCA9685(i2c)
servos = pca9685_servo.Servos(pca)

as5048_cs = pyb.Pin("X5", pyb.Pin.OUT_PP)
as5048_cs.high()
as5048_spi=pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER,prescaler=8,bits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs)

pca.duty(5,260)
max_angle = 0
min_angle = 1000
while True:
    angle = mag.read_angle()
    if angle > max_angle:
        max_angle = angle
    elif angle < min_angle:
        min_angle = angle
    print(max_angle, min_angle, angle)
    time.sleep(0.01)
