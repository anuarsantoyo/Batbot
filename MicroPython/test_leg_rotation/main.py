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
leg_y = 90
leg_y_amplitude = 30
leg_x = 40  # 50-140
leg_x_amplitude = 30
ellipse_angle = 1
motor = 260


# 50-160 extended-folded
#servos.position(ATTACK, attack_angle)
servos.position(FOLDER, extended)
pca.duty(FLAPPER,200)
time.sleep(1)

pca.duty(FLAPPER, motor)

import utime
'''
# Used to find max and min angle after 3 seconds of flapping
def measure_angle_for_1_second():
    start_time = utime.ticks_ms()
    max_angle = float('-inf')
    min_angle = float('inf')

    while utime.ticks_diff(utime.ticks_ms(), start_time) < 3000:
        angle = mag.read_angle()  # replace with your method of reading the angle
        max_angle = max(max_angle, angle)
        min_angle = min(min_angle, angle)
        time.sleep(0.001)

    return max_angle, min_angle
utime.sleep(1)
angle_max, angle_min = measure_angle_for_1_second()'''

angles = []


i=0
time.sleep(1)
start_time = utime.ticks_ms()
while utime.ticks_diff(utime.ticks_ms(), start_time) < 1000:
    i += 1
    time.sleep(0.001)
    if i<250:
        angle = mag.read_angle()
        angles.append(angle)
    else:
        break
    
angles = angles[-150:]
angle_max = max(angles)
angle_min = min(angles)
print('Max angle:', angle_max)
print('Min angle:', angle_min)

old_angle = mag.read_angle()  # Used to calculate derivative which gives stroke direction
start_time = utime.ticks_ms()

while True:
    time.sleep(0.001)
    
    # Used to slowly increase motor
    '''if utime.ticks_diff(utime.ticks_ms(), start_time) > 3000:
        time.sleep(0.001)
        start_time = utime.ticks_ms()
        motor += 1
        print(motor)
        if motor>285:
            motor=200
        pca.duty(FLAPPER, motor)'''

        
    
    new_angle = mag.read_angle()
    angles.append(new_angle)
    del angles[0]
    angle_max = max(angles)
    angle_min = min(angles)
    
    upward = new_angle < old_angle  # Calculate wing beat direction
    
    
    cyc = 1 - (new_angle-angle_min)/(angle_max-angle_min) # down:0 up:1
            
    if upward:
        pi_cyc = math.pi*cyc
    else:
        pi_cyc = 2*math.pi - math.pi*cyc

    
    if cyc>0.6:
        fold = extended # after half way up star extending
    elif cyc<0.15:
        fold = folded # 20% before reaching down start folding
    elif upward:
        fold = folded
    else:
        fold = extended # down-stroke extend
        
    servos.position(FOLDER, fold)
    old_angle = new_angle
    print(cyc)#, new_angle)
    
    y_theta = leg_y - leg_y_amplitude*math.cos(pi_cyc)        
    if y_theta < 30:
        y_theta = 30
    elif y_theta > 150:
        y_theta = 150
    LL_y_angle = y_theta
    RL_y_angle =  180 - y_theta
        
    x_theta = leg_x + leg_x_amplitude*math.sin(pi_cyc + 2*math.pi*ellipse_angle)
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
    
    #servos.position(ATTACK, attack_angle)
    pca.duty(FLAPPER, motor)