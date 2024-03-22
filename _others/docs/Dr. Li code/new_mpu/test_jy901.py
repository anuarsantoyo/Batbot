from machine import SoftI2C
import jy901
import time


#iic2=SoftI2C(scl='Y9',sda='Y10',freq=150000)
iic1=SoftI2C(scl='X9',sda='X10',freq=150000)

JY901=jy901.JY901(iic1)

while True:
    print(JY901.data_read())
    time.sleep(0.1)


