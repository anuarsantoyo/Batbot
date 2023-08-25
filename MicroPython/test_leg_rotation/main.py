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

cyc = 0
direction = 1

# Parameters
leg_y = 90
leg_y_amplitude = 0
leg_x = 40  # 50-140
leg_x_amplitude = 0
ellipse_angle = 0.5
attack_angle = 90
flapper = 110
motor = 190

# 50-160 extended-folded
servos.position(ATTACK, attack_angle)
pca.duty(FLAPPER, flapper)

while True:
    time.sleep(0.001)
    cyc += 0.005*direction
    if cyc>1:
        direction = -1
    elif cyc<0:
        direction = 1
    
    if direction == 1:
        pi_cyc = math.pi*cyc
    else:
        pi_cyc = 2*math.pi - math.pi*cyc
        
    pi_cyc *= -1
    
    if cyc>0.5:
        fold = 50 # after half way up star extending
    elif cyc<0.2:
        fold = 160 # 20% before reaching down start folding
    #elif old_angle<new_angle:
    #    fold = 130 # up-stroke folding
    elif direction == 1:
        fold = 160
    else:
        fold = 50 # down-stroke extend
        
    fold = 50  # TODO: remove
    servos.position(FOLDER, fold)

    print(cyc)
    
    y_theta = leg_y + leg_y_amplitude*math.sin(-pi_cyc)        
    if y_theta<30:
        y_theta = 30
    elif y_theta > 150:
        y_theta = 150
    LL_y_angle = y_theta
    RL_y_angle =  180 - y_theta
        
    x_theta = leg_x + leg_x_amplitude*math.sin(ellipse_angle*math.pi-pi_cyc)
    if x_theta<40:
        x_theta = 40
    elif x_theta > 120:
        x_theta = 120
    RL_x_angle =  x_theta
    LL_x_angle = 180 - RL_x_angle
    
    servos.position(LL_y, LL_y_angle)
    servos.position(RL_y, RL_y_angle)
    
    servos.position(RL_x, RL_x_angle)
    servos.position(LL_x, LL_x_angle)
    
    servos.position(ATTACK, attack_angle)
    pca.duty(FLAPPER, motor)