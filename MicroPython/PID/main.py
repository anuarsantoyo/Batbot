import pyb
import os
import math
from machine import I2C,SoftI2C
import time
import jy901
import as5048a
import pid
import pca9685
import pca9685_servo
import micropython
from micropython import const
import reciever
import servo_angle

micropython.opt_level(3)

#iic接口及iic设备初始化
iic1=SoftI2C(scl='X9',sda='X10',freq=150000)
imu=jy901.JY901(iic1)
pca=pca9685.PCA9685(iic1)
servos=pca9685_servo.Servos(pca)

reciever_UART=pyb.UART(3)     
reciever_UART.init(115200)
recv_decoder=reciever.Reciever_decoder()
recv_channels=[0]*16



RL_x = 0
RL_y = 1
LL_x = 2
LL_y = 3
FLAPPER = 4
legx_open = 90
legx_closed = 80


def linear_interp(x, x0, y0, x1, y1): # if x is x0 then give y0 back, if x1 then y1
    y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    return max(min(y1,y),y0)

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, sample_time):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.sample_time = sample_time

    def compute(self, input_value):
        # Calculate error
        error = self.setpoint - input_value

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * self.sample_time
        I = self.Ki * self.integral

        # Derivative term
        D = self.Kd * (error - self.prev_error) / self.sample_time

        # Total output
        output = P + I + D

        # Save error for next loop
        self.prev_error = error

        return output
    
    
class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []

    def update(self, value):
        if len(self.values) >= self.window_size:
            self.values.pop(0)
        self.values.append(value)
        return sum(self.values) / len(self.values)    
    
    
# PID stuff
sample_time = 0.01
k_roll_old = 0
k_pitch_old = 0

#window_filter_x = MovingAverageFilter(10) # Example window size
#window_filter_z = MovingAverageFilter(10) # Example window size
time.sleep(1)

while True:
    time.sleep(0.01)
    recv_nbytes=reciever_UART.any()
    if recv_nbytes!=0:
        
        #initilaze pid if new k value is found
        k_roll_new = recv_channels[5]
        k_pitch_new = recv_channels[6]
        
        if k_roll_new != k_roll_old:
            k_roll_old = k_roll_new
            kp = k_roll_old/1342  # Normalize 0-1
            pid_roll = PID(Kp=-1.5*kp, Ki=0, Kd=0, setpoint=-90, sample_time=sample_time)
            print("Changed roll k: ", kp)
        
        if k_pitch_new != k_pitch_old:
            k_pitch_old = k_pitch_new
            kp = k_pitch_old/1342  # Normalize 0-1
            pid_pitch = PID(Kp=-0.75*kp, Ki=0, Kd=0, setpoint=-20, sample_time=sample_time)
            print("Changed pitch k: ", kp)
        
        
        recv_data=reciever_UART.read(recv_nbytes)
        for i in recv_data:
            recv_decoder.passin(i)
        recv_channels=recv_decoder.get_channels_data()
        use_pid = recv_channels[8] == 0
        motor_raw = recv_channels[2] #0-1342 down-up
        motor = linear_interp(motor_raw, 0, 230, 1342, 400)
        pca.duty(FLAPPER, motor)
            
        pitch_raw = recv_channels[1] #0-671-1342 up-middle-down
        roll_raw = recv_channels[3] #0-671-1342 left-middle-rigth
        
        pitch_neutral = linear_interp(pitch_raw, 1342, 60, 0, 120)
        roll_neutral = linear_interp(roll_raw, 1342, -30, 0, 30)
        #print(pitch_neutral, roll_neutral)

        # Read the sensor value
        sensor = imu.data_read()
        sensor_pitch = sensor['Pth']
        sensor_roll = sensor['Rol']
        #sensor_pitch = window_filter_x.update(sensor['Pth'])
        #sensor_roll = window_filter_z.update(sensor['Rol'])
        #print(sensor_mpu['AcZ'])
        
        # Compute PID output
        pitch = pitch_neutral 
        roll = roll_neutral 
        if use_pid:
            pitch += int(pid_pitch.compute(sensor_pitch))
            roll += int(pid_roll.compute(sensor_roll))
        
        # Control the actuator
        y_theta = pitch
        LL_y_angle = (y_theta + roll)
        RL_y_angle = 165 - (y_theta - roll)
        
        x_theta = legx_closed
        RL_x_angle = x_theta
        LL_x_angle = 170 - x_theta
   
        servos.position(RL_x, RL_x_angle)
        servos.position(LL_x, LL_x_angle)
        servos.position(LL_y, LL_y_angle)
        servos.position(RL_y, RL_y_angle)
