import pyb
import os
import math
from machine import I2C,SoftI2C
import time
import mpu6050
import as5048a
import pid
import pca9685
import pca9685_servo
import micropython
from micropython import const
import reciever
import servo_angle

micropython.opt_level(3)
LOG=False

#iic接口及iic设备初始化
iic1=SoftI2C(scl='X9',sda='X10',freq=150000)
mpu=mpu6050.MPU6050(iic1)
pca=pca9685.PCA9685(iic1)
servos=pca9685_servo.Servos(pca)

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, sample_time):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.prev_error = 0
        self.integral = 0

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
    
pid = PID(Kp=-0.0001, Ki=-0.01, Kd=-0.00001, setpoint=0, sample_time=0.1)
servos.position(0, 60)
time.sleep(1)
while True:
    # Read the sensor value
    sensor_x = mpu.get_values()['AcX']
    print(sensor_x)
    # Compute PID output
    output = pid.compute(sensor_x)

    # Control the actuator
    servos.position(0, output)
    time.sleep(pid.sample_time)