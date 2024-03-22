import pyb
import os
import math
from machine import I2C,SoftI2C
import time
#import mpu6050
import jy901
import as5048a
import algorithm
import pca9685
import pca9685_servo
import micropython
from micropython import const
import reciever
import servo_angle
import gc

#iic接口及iic设备初始化
iic1=SoftI2C(scl='X9',sda='X10',freq=400000)
#mpu=mpu6050.MPU6050(iic1)
imu=jy901.JY901(iic1)

while True:
    print(imu.data_read())
    time.sleep(0.1)




