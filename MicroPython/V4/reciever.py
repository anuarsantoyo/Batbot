import pyb


class Reciever_decoder():
    def __init__(self):
        self.recv_buf=[0]*32
        self.flags=0
        self.recv_cnt=0
    

    def passin(self,data):
        if self.recv_cnt==0:
            if data!=0x0f:
                return  #丢弃异常帧头
            else:
                self.recv_cnt+=1
        elif self.recv_cnt<=32 and self.recv_cnt>0:
            self.recv_buf[self.recv_cnt-1]=data
            self.recv_cnt+=1
        elif self.recv_cnt==33:
            self.flags=data
            self.recv_cnt+=1
        else:
            self.recv_cnt=0
    
    def get_channels_data(self):
        channels=[0]*16
        for i in range(0,32):
            if i%2==0:
                channels[int(i/2)]=self.recv_buf[i]<<8
            else:
                channels[int((i-1)/2)]|=self.recv_buf[i]
                channels[int((i-1)/2)]-=353
        return channels
    
    def get_flags(self):
        return self.flags






