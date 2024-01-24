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

leg_extended = 40
leg_folded = 80 # {0:40, 671:80, 1342:120}

# down left, up left, down right, up right
case = {0: [leg_extended,leg_folded,leg_extended,leg_folded], # forward
        1: [leg_folded,leg_extended,leg_folded,leg_extended], # backward
        2: [leg_extended,leg_extended,leg_folded,leg_folded], # right
        3: [leg_folded,leg_folded,leg_extended,leg_extended], # left
        4: [leg_extended,leg_folded,leg_folded,leg_extended], # counter clock
        5: [leg_folded,leg_extended,leg_extended,leg_folded], # clockwise
        6: [leg_extended,leg_extended,leg_extended,leg_extended]} # Default  
        


def leg_logic(case_nr, upward):
    stroke = 1 if upward else 0
    left_leg = case[case_nr][stroke]
    right_leg = case[case_nr][stroke+2]
    return left_leg, right_leg

def case_reciever(recv_channels):
    if recv_channels[1] == 0: # up (rigth joystic)
        return 0 # forward
    elif recv_channels[1] == 1342: # down (rigth joystic)
        return 1 # backward
    elif recv_channels[3] == 1342: # right (rigth joystic)
        return 2 # right
    elif recv_channels[3] == 0: # left (rigth joystic)
        return 3 # left
    elif recv_channels[4] == 0: # down (left back lever)
        return 4 # counter clock wise
    elif recv_channels[4] == 1342: # up (left back lever)
        return 5 # clock wise
    else:
        return 6 # default
    
# Constants

RL_x = 0
RL_y = 1
LL_x = 2
LL_y = 3
FOLDER = 4
FLAPPER = 5
extended = 50


# Initilaization

time.sleep(3)
print('Initializing...')
servos.position(FOLDER, extended)
pca.duty(FLAPPER, 200)
time.sleep(3)
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
pca.duty(FLAPPER, 200)



old_angle = mag.read_angle()
while True:
    time.sleep(0.1)
    recv_nbytes=reciever_UART.any()
    if recv_nbytes!=0:
        
        recv_data=reciever_UART.read(recv_nbytes)
        for i in recv_data:
            recv_decoder.passin(i)
        recv_channels=recv_decoder.get_channels_data()
        #print(recv_channels)
        
        motor_raw = recv_channels[2] #0-1342 down-up
        motor = linear_interp(motor_raw, 0, 250, 1342, 290)
        pca.duty(FLAPPER, motor)
        
        new_angle = mag.read_angle()
        angles.append(new_angle)
        del angles[0]
        angle_max = max(angles)
        angle_min = min(angles)

        fold = extended
        servos.position(FOLDER, fold)
        
        upward = new_angle > old_angle  # Calculate wing beat direction       
        print(new_angle)
        case_nr = case_reciever(recv_channels)
        x_theta_R, x_theta_L = leg_logic(case_nr, upward)
        
        RL_x_angle = x_theta_R
        LL_x_angle = 180 - x_theta_L
        
        y_theta = 90
        LL_y_angle = y_theta
        RL_y_angle = 180 - y_theta

        servos.position(RL_x, RL_x_angle)
        servos.position(LL_x, LL_x_angle)
        servos.position(LL_y, LL_y_angle)
        servos.position(RL_y, RL_y_angle)
        
        old_angle = new_angle
        
            
        


            

    




