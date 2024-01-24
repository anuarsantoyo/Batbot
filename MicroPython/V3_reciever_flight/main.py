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
FLAPPER = 5
leg_extended = 50

# Initilaization

time.sleep(3)
print('Initializing...')
pca.duty(FLAPPER, 200)
time.sleep(1)


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
        motor = linear_interp(motor_raw, 0, 240, 1342, 400)
        pca.duty(FLAPPER, motor)

            
        pitch_raw = recv_channels[1] #0-671-1342 up-middle-down
        roll_raw = recv_channels[3] #0-671-1342 left-middle-rigth
        yaw_raw = recv_channels[0]
        
        pitch = linear_interp(pitch_raw, 1342, 30, 0, 150)
        roll = linear_interp(roll_raw, 0, -30, 1342, 30)
        yaw = linear_interp(yaw_raw, 0, -50, 1342, 50)
       
        y_theta = pitch
        LL_y_angle = (y_theta + roll)
        RL_y_angle = 180 - (y_theta - roll)
        
        if yaw>0:
            x_theta = leg_extended + yaw
            RL_x_angle = x_theta
            LL_x_angle = 180 - leg_extended
        
        else:
            x_theta = leg_extended - yaw
            RL_x_angle = leg_extended
            LL_x_angle = 180 - x_theta
                  
            
        servos.position(RL_x, RL_x_angle)
        servos.position(LL_x, LL_x_angle)
        servos.position(LL_y, LL_y_angle)
        servos.position(RL_y, RL_y_angle)
            
        


            

    




