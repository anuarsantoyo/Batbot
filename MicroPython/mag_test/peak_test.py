import pyb
import machine
import pca9685
import pca9685_servo
import as5048a
import time
import math
RL_x = 0
RL_y = 1
LL_x = 2
LL_y = 3
FOLDER = 4
FLAPPER = 5
ATTACK = 6
folded = 160
extended = 50

i2c = machine.SoftI2C("X9","X10",freq=100000)
pca = pca9685.PCA9685(i2c)
servos = pca9685_servo.Servos(pca)

as5048_cs = pyb.Pin("X5", pyb.Pin.OUT_PP)
as5048_cs.high()
as5048_spi=pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER,prescaler=8,bits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs)
pca.duty(FLAPPER,200)
time.sleep(3)
pca.duty(FLAPPER,260)
maxes = []
mines = []
angle_0 = 0
angle_1 = 0
angle_2 = 0

for i in range(500):
    angle_0 = angle_1
    angle_1 = angle_2
    angle_2 = mag.read_angle()
    if (angle_1>angle_0) and (angle_1>angle_2):
        print('A',angle_1)
        maxes.append(angle_1)
    elif (angle_1<angle_0) and (angle_1<angle_2):
        print('B',angle_1)
        mines.append(angle_1)
    time.sleep(0.01)

def median(lst):
    sorted_lst = sorted(lst)
    lst_len = len(sorted_lst)
    
    # If the list has an odd number of items, return the middle one
    if lst_len % 2 == 1:
        return sorted_lst[lst_len // 2]
    # If the list has an even number of items, return the average of the two middle ones
    else:
        left_mid = sorted_lst[(lst_len - 1) // 2]
        right_mid = sorted_lst[lst_len // 2]
        return (left_mid + right_mid) / 2

print(median(maxes))
print(median(mines))
