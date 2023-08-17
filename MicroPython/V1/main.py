import pyb
import machine
import pca9685
import pca9685_servo
import as5048a
import utime


# Configuration
i2c = machine.SoftI2C("X9","X10",freq=100000)
pca = pca9685.PCA9685(i2c)
servos = pca9685_servo.Servos(pca)
as5048_cs = pyb.Pin("X3", pyb.Pin.OUT_PP)
as5048_cs.high()
as5048_spi=pyb.SPI(1)
as5048_spi.init(mode=pyb.SPI.MASTER,prescaler=8,bits=8)
mag = as5048a.AS5048A(as5048_spi, as5048_cs) # readings from 70-110

# Constants
LEFT_LEG = 0
RIGHT_LEG = 1
FOLDER = 2
FLAPPER = 3
ATTACK_ANGLE = 4

uart = pyb.UART(3, 115200)

while True:
    response = uart.read()  # check what happens if several commands are sent
    if response:
        data_str = response.decode('utf-8')
        numbers = data_str.split(',')
        motor, attack_angle, leg_angle, leg_amplitude = [int(num) for num in numbers]      
# Test#motor = 270#attack_angle = 110#leg_angle = 150#leg_amplitude = 30
        pca.duty(FLAPPER, motor)
        servos.position(ATTACK_ANGLE, attack_angle)
        
        start_time = utime.ticks_ms()
        duration = 10000  # 10 seconds in deciseconds
        old_angle = mag.read_angle()  #Used to calculate derivative which gives stroke direction
        
        while utime.ticks_diff(utime.ticks_ms(), start_time) < duration:
            utime.sleep(0.001)
            new_angle = mag.read_angle()
            cyc = (new_angle-70)/(110-70)  #down:0 up:1
            if cyc>0.5:
                fold = 30 # after half way up star extending
            elif cyc<0.2:
                fold = 130 # 20% before reaching down start folding
            elif old_angle<new_angle:
                fold = 130 # up-stroke folding
            else:
                fold = 130 # down-stroke extend

            # legs down (neutral_state-amplitude) - up(neutral_state+amplitude): 80-170
            legs = (leg_angle-leg_amplitude)*(1-cyc) + (leg_angle+leg_amplitude)*cyc;
            if legs<80:
              legs = 80
            elif legs>170:
              legs = 170
            
            servos.position(LEFT_LEG, degrees = legs)
            servos.position(RIGHT_LEG, degrees = legs)
            servos.position(FOLDER, degrees = fold) #Extended 30, folded 130
            old_angle = new_angle
            
        uart.read() # in case data was sent while in the loop it is deleted
        pca.duty(FLAPPER, 250)  # Stop flapper
        servos.position(FOLDER, degrees = 130)















