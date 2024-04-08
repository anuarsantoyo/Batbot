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
import utime
import gc


micropython.opt_level(3)

#iic接口及iic设备初始化
iic1=SoftI2C(scl='X9',sda='X10',freq=150000)
pca=pca9685.PCA9685(iic1)
servos=pca9685_servo.Servos(pca)
as5048_cs = pyb.Pin("X4", pyb.Pin.OUT_PP)  # For magnetic encoder (wing angle measurement)
as5048_cs.high()
as5048_spi = pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER, prescaler=8, bits=8)
mag_sensor = as5048a.AS5048A(as5048_spi, as5048_cs)
uart = pyb.UART(3, 115200)  # Wireless uart to send commands.


RL_x = 0
RL_y = 1
LL_x = 2
LL_y = 3
FLAPPER = 4 # from 240
FOLDER = 5 # 40-140:extended-folded
folded = 140
extended = 40

def move(motor, x_theta, y_theta, fold):
    # Control the actuator
    #y_theta = 0 # -90 : 90
    LL_y_angle = y_theta + 100
    RL_y_angle = 80 - y_theta 
    
    #x_theta = 0 # 0-180
    RL_x_angle = x_theta + 48
    LL_x_angle = 135 - x_theta
    
    
    pca.duty(FLAPPER, motor)
    servos.position(FOLDER, fold)
    servos.position(RL_x, RL_x_angle)
    servos.position(LL_x, LL_x_angle)
    servos.position(LL_y, LL_y_angle)
    servos.position(RL_y, RL_y_angle)
    

class CycCalculation:
    def __init__(self, mag_sensor):
        self.angles = [0] * 200  # Initialize the list of angles with 500 zeros.
        self.mag = mag_sensor  # This should be your magnetometer object.
        self.old_angle = self.mag.read_angle()  # Store the initial angle read from the magnetometer.
        self.angle_max = max(self.angles)  # The maximum angle in the current list of angles.
        self.angle_min = min(self.angles)  # The minimum angle in the current list of angles.

    def update(self):
        new_angle = self.mag.read_angle()
        self.angles.append(new_angle)
        del self.angles[0]
        self.angle_max = max(self.angles)
        self.angle_min = min(self.angles)
        
        delta_y1 = self.angles[-1] - self.angles[-2]  # Change between first and second point.
        delta_y2 = self.angles[-2] - self.angles[-3]  # Change between second and third point.
        delta_y3 = self.angles[-3] - self.angles[-4]
        
        average_delta_y = (delta_y1 + delta_y2 + delta_y3) / 3

        # Since x is uniform, we can assume delta_x as 1.
        upward = average_delta_y > 0  # Determine wing beat direction.
        cyc = 2 * ((new_angle - self.angle_min) / (self.angle_max - self.angle_min)) - 1  # Normalize cyc value between -1 and 1.
        
        # Calculate cyc_pi based on the direction indicated by 'upward'.
        if upward:
            cyc_pi = (cyc + 1) / 2 * math.pi  # Map cyc from 0 to 1 to 0 to pi.
        else:
            cyc_pi = math.pi * (2 - ((cyc + 1) / 2))  # Map cyc from 0 to 1 to pi to 2*pi.

        self.old_angle = new_angle
        
        return upward, cyc_pi, cyc
    
def is_leg_upward(cyc_pi, range_start):
    """
    Check if cyc_pi is within a cyclic range of width pi,
    starting from range_start, considering the circular nature of angles.

    :param cyc_pi: The angle in radians, normalized to [0, 2*pi).
    :param range_start: The start of the range, normalized to [0, 2*pi).
    :return: True if cyc_pi is within the range, False otherwise.
    """

    range_start = range_start % (2 * math.pi)
    
    # Calculate the end of the range
    range_end = (range_start + math.pi) % (2 * math.pi)
    
    # Determine if cyc_pi is within the range
    if range_start < range_end:
        return range_start <= cyc_pi < range_end
    else:
        # The range wraps around, so cyc_pi is within the range if it's
        # either greater than the start or less than the end.
        return cyc_pi >= range_start or cyc_pi < range_end

cyc_calculation = CycCalculation(mag_sensor)
motor = 280
pca.duty(FLAPPER, 200)
time.sleep(1)


pca.duty(FLAPPER, motor)

'''
print('Calculating min/max...')
start_time = utime.ticks_ms()
while utime.ticks_diff(utime.ticks_ms(), start_time) < 3000:
    cyc_calculation.update()
    time.sleep(0.01)
print('Done!')
pca.duty(FLAPPER, 200)
'''
while True:
    response = uart.read()  # read command
    if response:
        print(response)
        pca.duty(FLAPPER, motor)
        data_str = response.decode('utf-8')
        numbers = data_str.split(',')
        x_amp, y_amp, y_mid, delay = [float(num) for num in numbers]
        
        duration = 6500
        start_time = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start_time) < duration:
        
            upward, cyc_pi, cyc = cyc_calculation.update() # Calculate cicle of wing
            leg_upward = is_leg_upward(cyc_pi, delay)
            x_theta = x_amp if leg_upward else 0
            fold = folded if upward else extended
            y_theta = y_mid - y_amp*math.cos(cyc_pi + delay)
            move(motor, x_theta, y_theta, fold)
            time.sleep(0.001)
            
        uart.read()  # in case data was sent while in the loop it is deleted
        #pca.duty(FLAPPER, 200)  # Turn off flapper

