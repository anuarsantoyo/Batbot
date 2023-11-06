import pyb


class pid():
    def __init__(self,Kp,Ki,Kd,pid_value,value_max=9999,value_min=-9999,use_inte=False):
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.pid_value=pid_value  #若使用积分环节则为初始值，若不使用积分则为零位值
        self.use_inte=use_inte    #是否使用积分环节
        self.value_max=value_max  #使用积分环节时的最大限位
        self.value_min=value_min  #使用积分环节时的最小限位
        self.integration=0
        self.last_value=0
    

    def calc(self,value,goal):
        self.integration+=value
        if self.use_inte:
            self.pid_value+=(value-goal)*self.Kp+self.integration*self.Ki+(value-self.last_value)*self.Kd
            if self.pid_value>self.value_max:
                self.pid_value=self.value_max
            if self.pid_value<self.value_min:
                self.pid_value=self.value_min
        else:
            ret=(value-goal)*self.Kp+self.integration*self.Ki+(value-self.last_value)*self.Kd+self.pid_value
        self.last_value=value
        if self.use_inte:
            return self.pid_value
        else:
            return ret