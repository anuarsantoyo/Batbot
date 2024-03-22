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
as5048_cs = pyb.Pin("X4", pyb.Pin.OUT_PP)  # For magnetic encoder (wing angle measurement)
as5048_cs.high()
as5048_spi = pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER, prescaler=8, bits=8)
reciever_UART=pyb.UART(3)       #Y10
reciever_UART.init(115200)
recv_decoder=reciever.Reciever_decoder()
recv_channels=[0]*16
mag = as5048a.AS5048A(as5048_spi, as5048_cs)

def linear_interp(x, x0, y0, x1, y1): # if x is x0 then give y0 back, if x1 then y1
    y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    return max(min(y1,y),y0)

# Constants
RL_x = 0
RL_y = 1
LL_x = 2
LL_y = 3
FLAPPER = 4
legx_open = 90
legx_closed = 40


time.sleep(3)
print('Initializing...')
pca.duty(FLAPPER, 200)
time.sleep(1)


angles = [0]*500
old_angle = mag.read_angle()
angle_max = max(angles)
angle_min = min(angles)

while True:
    time.sleep(0.001)
    recv_nbytes=reciever_UART.any()
    if recv_nbytes!=0:
        recv_data=reciever_UART.read(recv_nbytes)
        for i in recv_data:
            recv_decoder.passin(i)
        recv_channels=recv_decoder.get_channels_data()
        #print(recv_channels)
        
        new_angle = mag.read_angle()
        angles.append(new_angle)
        del angles[0]
        angle_max = max(angles)
        angle_min = min(angles)
        
        
        delta_y1 = angles[-1] - angles[-2]  # Change between first and second point
        delta_y2 = angles[-2] - angles[-3]  # Change between second and third point
        delta_y3 = angles[-3] - angles[-4] 
        average_delta_y = (delta_y1 + delta_y2 + delta_y3) / 3

        # Since x is uniform, we can assume delta_x as 1
        upward = average_delta_y > 0 
        #upward = new_angle > old_angle  # Calculate wing beat direction
        
        cyc = 2*((new_angle - angle_min) / (angle_max - angle_min)) - 1  # down:-1 up:1
        print(old_angle)
        old_angle = new_angle
        
        C_LY = recv_channels[2] #0-1342 down-up
        motor = linear_interp(C_LY, 0, 200, 1342, 400)
        pca.duty(FLAPPER, motor)
            
        C_RY = recv_channels[1] # 0-671-1342 up-middle-down
        #C_RX = recv_channels[3] # 0-671-1342 left-middle-rigth
        #C_LX = recv_channels[0] # 0-671-1342 left-middle-rigth
        
        y_amp = linear_interp(C_RY, 1342, -40, 0, 40)
        forward = y_amp>0
        y_theta = 90 + abs(y_amp)*cyc
        
        if y_amp == 0:
            x_theta = legx_closed
        elif forward:
            x_theta = legx_open if upward else legx_closed
        elif not forward:
            x_theta = legx_open if not upward else legx_closed
        
        
        LL_y_angle = y_theta
        RL_y_angle = 180 - y_theta
        RL_x_angle = x_theta
        LL_x_angle = 180 - RL_x_angle
            
        servos.position(RL_x, RL_x_angle)
        servos.position(LL_x, LL_x_angle)
        servos.position(LL_y, LL_y_angle)
        servos.position(RL_y, RL_y_angle)
