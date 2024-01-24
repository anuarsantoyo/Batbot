import pyb


class AS5048A():
    def __init__(self,spi,cs_pin,offset=0):
        self.spi=spi
        self.cs_pin=cs_pin   #注意：cs引脚初始状态为高
        self.offset=offset
    


    def read_angle(self):
        self.cs_pin.low()
        self.spi.send(bytes([0xff,0xff]),timeout=5)
        self.cs_pin.high()
        self.cs_pin.low()
        ret=self.spi.recv(2,timeout=5)
        self.cs_pin.high()
        #print(ret)  #test
        angle=(ret[1]|(ret[0]<<8))*0.022
        angle+=self.offset
        while angle>360:
            angle-=360
        while angle<0:
            angle+=360
        return angle


