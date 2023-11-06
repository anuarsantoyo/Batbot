import pyb
import machine
import pca9685
import pca9685_servo
import as5048a
import utime
from micropython import const
import math
import time

# Configuration
i2c = machine.SoftI2C("X9","X10",freq=100000)  # For PCA
pca = pca9685.PCA9685(i2c)
servos = pca9685_servo.Servos(pca)
as5048_cs = pyb.Pin("X5", pyb.Pin.OUT_PP)  # For magnetic encoder (wing angle measurement)
as5048_cs.high()
as5048_spi = pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER, prescaler=8, bits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs)  # readings from 235-194 -> down-up
uart = pyb.UART(3, 115200)  # Wireless uart to send commands.

# Constants
RL_x = 0
RL_y = 1
LL_x = 2
LL_y = 3
FOLDER = 4
FLAPPER = 5
folded = 160
extended = 50

# Calculate max and min angles (for some weir reason they consantly change)
#pca.duty(FLAPPER,200)
#time.sleep(3)
#pca.duty(FLAPPER,260)
#maxes = []
#mines = []
#angle_0 = 0
#angle_1 = 0
#angle_2 = 0

#for i in range(500):
#    angle_0 = angle_1
#    angle_1 = angle_2
#    angle_2 = mag.read_angle()
#    if (angle_1>angle_0) and (angle_1>angle_2):
#        maxes.append(angle_1)
#    elif (angle_1<angle_0) and (angle_1<angle_2):
#        mines.append(angle_1)
#    time.sleep(0.01)

#def median(lst):
#    sorted_lst = sorted(lst)
#    lst_len = len(sorted_lst)
    
    # If the list has an odd number of items, return the middle one
#    if lst_len % 2 == 1:
#        return sorted_lst[lst_len // 2]
    # If the list has an even number of items, return the average of the two middle ones
#    else:
#        left_mid = sorted_lst[(lst_len - 1) // 2]
#        right_mid = sorted_lst[lst_len // 2]
#        return (left_mid + right_mid) / 2

#angle_max = median(maxes)
#angle_min = median(mines)

angle_max = 331
angle_min = 299

# Motors initialization
pca.duty(FLAPPER, 260)
#servos.position(ATTACK, 120)
servos.position(FOLDER, extended)
utime.sleep(3)

while True:
    response = uart.read()  # read command
    if response:
        print(response)
        data_str = response.decode('utf-8')
        numbers = data_str.split(',')
        motor, attack_angle, leg_x, leg_y, leg_x_amplitude, leg_y_amplitude, ellipse_angle \
            = [float(num) for num in numbers]  # Extract command

        pca.duty(FLAPPER, motor)
        #servos.position(ATTACK, attack_angle)
        
        start_time = utime.ticks_ms()
        duration = 6500  # 7.5 seconds

        old_angle = mag.read_angle()  # Used to calculate derivative which gives stroke direction
        while utime.ticks_diff(utime.ticks_ms(), start_time) < duration:  # Run as long as stated experiment duration
            utime.sleep(0.001)  #  Used to stabiize the loop, if not added time measurement fails.
            new_angle = mag.read_angle()
            upward = new_angle > old_angle  # Calculate wing beat direction
            cyc = (new_angle-angle_min)/(angle_max-angle_min)  # down:0 up:1
            print(new_angle, cyc)
            if upward:
                pi_cyc = math.pi * cyc  # [0,pi]
            else:
                pi_cyc = 2 * math.pi - math.pi * cyc  # [pi, 2pi]

            if cyc > 0.5:
                fold = extended  # after halfway up, start extending
            elif cyc < 0.2:
                fold = folded  # 20% before reaching down, start folding
            elif upward:
                fold = folded  #  up-stroke fold
            else:
                fold = extended  # down-stroke extend

            #servos.position(FOLDER, extended) # uncomment if you want to disable folding
            servos.position(FOLDER, fold)
            old_angle = new_angle

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

            #servos.position(ATTACK, attack_angle)
            pca.duty(FLAPPER, motor)

        uart.read()  # in case data was sent while in the loop it is deleted
        pca.duty(FLAPPER, 200)  # Turn off flapper

