import pyb
import machine
import pca9685
import pca9685_servo
import as5048a
import utime
from micropython import const
import math
import time
import reciever

# Configuration
i2c = machine.SoftI2C("X9","X10",freq=100000)  # For PCA
pca = pca9685.PCA9685(i2c)
servos = pca9685_servo.Servos(pca)
as5048_cs = pyb.Pin("X5", pyb.Pin.OUT_PP)  # For magnetic encoder (wing angle measurement)
as5048_cs.high()
as5048_spi = pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER, prescaler=8, bits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs)  # readings from 235-194 -> down-up
reciever_UART=pyb.UART(3)       #Y10
reciever_UART.init(115200)
recv_decoder=reciever.Reciever_decoder()
recv_channels=[0]*16

def linear_interp(x, x0, y0, x1, y1):
    y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    return max(min(y1,y),y0)


# Constants

RL_x = 0
RL_y = 1
LL_x = 2
LL_y = 3
FOLDER = 4
FLAPPER = 5
folded = {1342:140, 0:50, 671:140}
extended = 50
leg_extended = 40
leg_folded = {0:40, 671:80, 1342:120}

# For Hovering
leg_y = 45  # [30, 150]
leg_y_amplitude = 20  # [0, 60]
leg_x = 57  # [40, 140]
leg_x_amplitude = 25  # [0, 50]
ellipse_angle = 0.9  # [0, 1]

# Initilaization

time.sleep(3)
print('Initializing...')
servos.position(FOLDER, extended)
pca.duty(FLAPPER, 200)
time.sleep(3)
pca.duty(FLAPPER, 250)
time.sleep(1)
pca.duty(FLAPPER, 200)
angles = [0]*200
'''time.sleep(3)
pca.duty(FLAPPER, 260)
time.sleep(1)
print('Iniitialized!')
angles = []
i = 0
print('Calculating min/max...')
start_time = utime.ticks_ms()

while utime.ticks_diff(utime.ticks_ms(), start_time) < 3000:
    i += 1
    time.sleep(0.01)
    if i < 200:
        angle = mag.read_angle()
        angles.append(angle)
    else:
        break

angle_max = max(angles)
angle_min = min(angles)
print('Calculated!')
print('Max angle:', angle_max)
print('Min angle:', angle_min)
pca.duty(FLAPPER, 200)'''



old_angle = mag.read_angle()
while True:
    time.sleep(0.1)
    recv_nbytes=reciever_UART.any()
    if recv_nbytes!=0:
        
        recv_data=reciever_UART.read(recv_nbytes)
        for i in recv_data:
            recv_decoder.passin(i)
        recv_channels=recv_decoder.get_channels_data()
        print(recv_channels)
        
        motor_raw = recv_channels[2] #0-1342 down-up
        motor = linear_interp(motor_raw, 0, 240, 1342, 380)
        pca.duty(FLAPPER, motor)
        
        new_angle = mag.read_angle()
        angles.append(new_angle)
        del angles[0]
        angle_max = max(angles)
        angle_min = min(angles)

        upward = new_angle > old_angle  # Calculate wing beat direction
        cyc = (new_angle - angle_min) / (angle_max - angle_min)  # down:0 up:1
        #print(cyc)

        if cyc > 0.6:
            fold = extended  # after half way up star extending
            leg = leg_extended
        elif cyc < 0.15:
            fold = folded[recv_channels[8]]# 20% before reaching down start folding
            leg = leg_folded[recv_channels[4]]
        elif upward:
            fold = folded[recv_channels[8]]
            leg = leg_folded[recv_channels[4]]
        else:
            fold = extended  # down-stroke extend
            leg = leg_extended

        servos.position(FOLDER, fold)
        
        old_angle = new_angle
        if upward:
            pi_cyc = math.pi * cyc  # [0,pi]
        else:
            pi_cyc = 2 * math.pi - math.pi * cyc  # [pi, 2pi]
        
        
        if recv_channels[7] == 1342:  # Hover mode

            y_theta = leg_y - leg_y_amplitude * math.cos(pi_cyc)  # Calculate angle from body plane to leg in vertical
            if y_theta < 30:  # Stops angle from exceeding limit values (also allowing straight trajectories in leg)
                y_theta = 30
            elif y_theta > 150:
                y_theta = 150
            LL_y_angle = y_theta
            RL_y_angle = 180 - y_theta  # Invert for right leg

            # Calculate angle from body plane to leg in vertical
            x_theta = leg_x + leg_x_amplitude * math.sin(pi_cyc + 2 * math.pi * ellipse_angle)
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
            
        
        if recv_channels[7] == 0:  # Flight mode
            x_theta = leg
            RL_x_angle = x_theta
            LL_x_angle = 180 - RL_x_angle
            servos.position(RL_x, RL_x_angle)
            servos.position(LL_x, LL_x_angle)
            
            pitch_raw = recv_channels[1] #0-671-1342 up-middle-down
            turn_raw = recv_channels[3] #0-671-1342 left-middle-rigth
            
            pitch = linear_interp(pitch_raw, 1342, 30, 0, 150)
            turn = linear_interp(turn_raw, 0, -30, 1342, 30)
            
            y_theta = pitch
            LL_y_angle = (y_theta + turn)
            RL_y_angle = 180 - (y_theta - turn)
            
            
            servos.position(LL_y, LL_y_angle)
            servos.position(RL_y, RL_y_angle)
            
        


            

    




