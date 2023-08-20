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
as5048_cs = pyb.Pin("X3", pyb.Pin.OUT_PP)
as5048_cs.high()
as5048_spi=pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER,prescaler=8,bits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs)

LL_x = 2
LL_y = 3
RL_x = 0
RL_y = 1
FOLDER = 4
ATTACK = 5

cyc = 0
direction = 1

# Parameters
leg_y = 90
leg_y_amplitude = 40
leg_x = 90  # 50-140
leg_x_amplitude = 40
ellipse_angle = 0.5
attack_angle = 130
# 50-160 extended-folded
servos.position(ATTACK, attack_angle)
time.sleep(3)

while True:
    time.sleep(0.0001)
    cyc += 0.015*direction
    if cyc>1:
        direction = -1
    elif cyc<0:
        direction = 1
    
    if direction == 1:
        pi_cyc = math.pi*cyc
    else:
        pi_cyc = 2*math.pi - math.pi*cyc
        
        
    if cyc>0.5:
        fold = 160 # after half way up star extending
    elif cyc<0.2:
        fold = 50 # 20% before reaching down start folding
    #elif old_angle<new_angle:
    #    fold = 130 # up-stroke folding
    elif direction == 1:
        fold = 50
    else:
        fold = 50 # down-stroke extend
    
    servos.position(FOLDER, fold)

    print(cyc)
    servos.position(LL_y, leg_y + leg_y_amplitude*math.sin(-pi_cyc))
    servos.position(RL_y, 180 - (leg_y + leg_y_amplitude*math.sin(-pi_cyc)))
    
    servos.position(RL_x, leg_x + leg_x_amplitude*math.sin(ellipse_angle*math.pi-pi_cyc))
    servos.position(LL_x, 180 - (leg_x + leg_x_amplitude*math.sin(ellipse_angle*math.pi-pi_cyc)))   


