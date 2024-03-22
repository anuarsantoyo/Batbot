from time import sleep


class MPU6050():
    def __init__(self, i2c, addr=0x68):
        self.iic = i2c
        self.addr = addr
        self.iic.start()
        self.iic.writeto(self.addr, bytearray([107, 0]))
        self.iic.stop()
        sleep(0.05)
        self.iic.start()
        self.iic.writeto(self.addr, bytearray([28, 0x10]))
        self.iic.stop()
        self.iic.start()
        self.iic.writeto(self.addr, bytearray([27, 0x10]))
        self.iic.stop()
        try:
            with open('mpu_cali_data.txt','r') as file:
                lines=file.readlines()
            self.offset={
                'AcX':-int(lines[0]),
                'AcY':-int(lines[1]),
                'AcZ':-int(lines[2]),
                'GyX':-int(lines[3]),
                'GyY':-int(lines[4]),
                'GyZ':-int(lines[5])
            }
        except:
            self.offset={'AcX':0,'AcY':0,'AcZ':0,'GyX':0,'GyY':0,'GyZ':0}
        

    def get_raw_values(self):
        self.iic.start()
        a = self.iic.readfrom_mem(self.addr, 0x3B, 14)
        self.iic.stop()
        return a

    def get_ints(self):
        b = self.get_raw_values()
        c = []
        for i in b:
            c.append(i)
        return c

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def get_values(self):
        raw_ints = self.get_raw_values()
        vals = {}
        vals["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1])+self.offset['AcX']
        vals["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3])+self.offset['AcY']
        vals["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5])+self.offset['AcZ']
        vals["Tmp"] = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
        vals["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9])  +self.offset['GyX']
        vals["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11])+self.offset['GyY']
        vals["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13])+self.offset['GyZ']
        return vals  # returned in range of Int16
        # -32768 to 32767

    def calibrate(self):
        cali_data={'AcX':0,'AcY':0,'AcZ':0,'GyX':0,'GyY':0,'GyZ':0}
        self.offset={'AcX':0,'AcY':0,'AcZ':0,'GyX':0,'GyY':0,'GyZ':0}
        for _ in range(100):
            data=self.get_values()
            cali_data['AcX']+=data['AcX']
            cali_data['AcY']+=data['AcY']
            cali_data['AcZ']+=data['AcZ']-4096
            cali_data['GyX']+=data['GyX']
            cali_data['GyY']+=data['GyY']
            cali_data['GyZ']+=data['GyZ']
            sleep(0.05)
        for i in cali_data:
            cali_data[i]/=100
        with open('mpu_cali_data.txt',mode='w') as file:
            file.write('%d\n'%cali_data['AcX'])
            file.write('%d\n'%cali_data['AcY'])
            file.write('%d\n'%cali_data['AcZ'])
            file.write('%d\n'%cali_data['GyX'])
            file.write('%d\n'%cali_data['GyY'])
            file.write('%d\n'%cali_data['GyZ'])

    def val_test(self):  # ONLY FOR TESTING! Also, fast reading sometimes crashes IIC
        while 1:
            print(self.get_values())
            sleep(0.05)
