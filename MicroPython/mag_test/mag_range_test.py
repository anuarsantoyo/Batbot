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

LL_x = 2
LL_y = 3
RL_x = 0
RL_y = 1
FOLDER = 4
FLAPPER = 5
ATTACK = 6
folded = 160
extended = 50

pca.duty(FLAPPER,260)
servos.position(ATTACK,120)
servos.position(FOLDER, folded)
time.sleep(3)
servos.position(FOLDER, extended)
start_time = time.time()

ranges = []
for motor in [260+i for i in range(10)]:
    max_angle = 0
    min_angle = 1000
    pca.duty(FLAPPER, motor)
    while True:
        now = time.time()
        diff = now-start_time
        if diff > 3:
            start_time = now
            break

        angle = mag.read_angle()
        if angle > max_angle:
            max_angle = angle
        elif angle < min_angle:
            min_angle = angle
        print(max_angle, min_angle, angle)
        time.sleep(0.01)
    ranges.append((min_angle, max_angle))
print(ranges)
