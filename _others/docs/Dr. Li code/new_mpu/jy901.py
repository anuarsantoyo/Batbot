import ustruct



class JY901():
    def __init__(self,iic,address=0x50):
        self.addr=address
        self.iic=iic

    def write_data(self, reg, data):
        self.iic.writeto_mem(self.addr,reg,bytearray(data))

    def read_data(self, reg, length):
        return self.iic.readfrom_mem(self.addr,reg,length)
    
    def hight_tozero(self):
        self.write_data(0x01,b'\x03\x00')

    def data_read_raw(self,pressure=True): #读取九(十)轴原始数据
        r_data = self.read_data(0x34,18)
        #print("data is {0}".format(list(r_data)))
        val = ustruct.unpack("<hhhhhhhhh", bytearray(r_data))

        retval={
            'AcX':val[0]/32768.0*16.0, #单位：G
            'AcY':val[1]/32768.0*16.0,
            'AcZ':val[2]/32768.0*16.0,
            'GyX':val[3]/32768.0*2000.0, #单位：°/s
            'GyY':val[4]/32768.0*2000.0,
            'GyZ':val[5]/32768.0*2000.0,
            'HlX':val[6]/32768.0,      #单位：mGs
            'HlY':val[7]/32768.0,
            'HlZ':val[8]/32768.0
        }
        if pressure:
            r_data = self.read_data(0x45,4)
            val=ustruct.unpack('<i',r_data)
            retval['Pr']=val[0]    #单位：Pa
        return retval
    
    def data_read(self,hight=True):
        r_data = self.read_data(0x3D,6)
        val = ustruct.unpack("<hhh", bytearray(r_data))
        retval={
            'Rol':val[0]/32768.0*180.0,  #单位：°
            'Pth':val[1]/32768.0*180.0,
            'Yaw':val[2]/32768.0*180.0
        }
        if hight:
            r_data = self.read_data(0x47,4)
            val=ustruct.unpack('<i',r_data)
            retval['Alt']=val[0]/100.0   #单位：m
        return retval
