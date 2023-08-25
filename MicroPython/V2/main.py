import pyb
import machine
import pca9685
import pca9685_servo
import as5048a
import utime
from micropython import const
import math


# Configuration
i2c = machine.SoftI2C("X9","X10",freq=100000)
pca = pca9685.PCA9685(i2c)
servos = pca9685_servo.Servos(pca)
as5048_cs = pyb.Pin("X5", pyb.Pin.OUT_PP)
as5048_cs.high()
as5048_spi = pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER, prescaler=8, äbits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs)  # readings from 70-110

# Constants
LL_x = 2
LL_y = 3
RL_x = 0
RL_y = 1
FOLDER = 4
FLAPPER = 5
ATTACK = 6
folded = 160
extended = 50

uart = pyb.UART(3, 115200)

# Motors initialization
pca.duty(FLAPPER, 100)
servos.position(ATTACK, 110)
#servos.position(FOLDER, 100)
#servos.position(RIGHT_LEG, 100)
#servos.position(LEFT_LEG, 100)
utime.sleep(3)

while True:
    response = uart.read()  # check what happens if several commands are sent
    if response:
        print(response)
        data_str = response.decode('utf-8')
        numbers = data_str.split(',')
        leg_y, leg_y_amplitude, leg_x, leg_x_amplitude, ellipse_angle, attack_angle, motor = [float(num) for num in numbers]
# Test#motor = 270#attack_angle = 110#leg_angle = 150#leg_amplitude = 30
        pca.duty(FLAPPER, motor)
        servos.position(ATTACK, attack_angle)
        
        start_time = utime.ticks_ms()
        duration = 10000  # 10 seconds in deciseconds
        old_angle = mag.read_angle()  #Used to calculate derivative which gives stroke direction
        
        while utime.ticks_diff(utime.ticks_ms(), start_time) < duration:
            utime.sleep(0.001)
            new_angle = mag.read_angle()
            upstroke = old_angle < new_angle
            cyc = (new_angle-70)/(110-70)  #down:0 up:1

            if cyc > 0.5:
                fold = extended  # after half_way up star extending
            elif cyc < 0.2:
                fold = folded  # 20% before reaching down start folding
            elif upstroke:
                fold = folded  # up-stroke folding
            else:
                fold = extended  # down-stroke extend
            servos.position(FOLDER, fold)

            if upstroke:
                pi_cyc = math.pi * cyc
            else:
                pi_cyc = 2 * math.pi - math.pi * cyc
            pi_cyc *= -1

            y_theta = leg_y + leg_y_amplitude * math.sin(-pi_cyc)
            if y_theta < 30:
                y_theta = 30
            elif y_theta > 150:
                y_theta = 150
            LL_y_angle = y_theta
            RL_y_angle = 180 - y_theta

            x_theta = leg_x + leg_x_amplitude * math.sin(ellipse_angle * math.pi - pi_cyc)
            if x_theta < 50:
                x_theta = 50
            elif x_theta > 120:
                x_theta = 120
            RL_x_angle = x_theta
            LL_x_angle = 180 - RL_x_angle

            servos.position(LL_y, LL_y_angle)
            servos.position(RL_y, RL_y_angle)

            servos.position(RL_x, RL_x_angle)
            servos.position(LL_x, LL_x_angle)
            old_angle = new_angle
            
        uart.read()  # in case data was sent while in the loop it is deleted
        pca.duty(FLAPPER, 100)  # Stop flapper
        servos.position(FOLDER, degrees=130)

