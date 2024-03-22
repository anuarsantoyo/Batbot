
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
        self.last_time=None
    

    @micropython.native
    def calc(self,time,value,goal):
        self.integration+=value
        if self.use_inte:
            if self.last_time is not None:
                self.pid_value+=(value-goal)*self.Kp+self.integration*self.Ki+(value-self.last_value)/(time-self.last_time)*self.Kd
            if self.pid_value>self.value_max:
                self.pid_value=self.value_max
            if self.pid_value<self.value_min:
                self.pid_value=self.value_min
        else:
            if self.last_time is not None:
                ret=(value-goal)*self.Kp+self.integration*self.Ki+(value-self.last_value)/(time-self.last_time)*self.Kd+self.pid_value
            else:
                ret=self.pid_value
        self.last_value=value
        self.last_time=time
        if self.use_inte:
            return self.pid_value
        else:
            return ret



class KalmanFilter():
    def __init__(self,Q,R,Kg,LastP=0,Now_P=0,out=0):
        #卡尔曼滤波初始化参数
        self.LastP=LastP#上次估算协方差
        self.Now_P=Now_P#当前估算协方差
        self.out=out#卡尔曼滤波器输出
        self.Kg=Kg#//卡尔曼增益
        self.Q=Q#过程噪声协方差
        self.R=R#观测噪声协方差
 
    
    def filt(self,mesureValue):
        #预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
        self.Now_P=self.LastP+self.Q
        #卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
        self.Kg=self.Now_P/(self.Now_P+self.R)
        #更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
        self.out=self.out+self.Kg*(mesureValue-self.out)
        #更新协方差方程: 本次的系统协方差付给 kfp->LastP 为下一次运算准备。
        self.LastP=(1-self.Kg)*self.Now_P
        return self.out 


class ComplementaryFilter():
    def __init__(self):
        self.last_attitude=0
        self.last_time=None

    @micropython.native
    def filt(self,gyro_angV,accel_angle,alpha,time=None,dtime=None):  #确保time*gyro_angV的单位与accel_angle相同
        if self.last_time is not None:
            if time is not None:
                attitude=(self.last_attitude+gyro_angV*(time-self.last_time))*(1-alpha)+accel_angle*alpha
                self.last_time=time
            elif dtime is not None:
                attitude=(self.last_attitude+gyro_angV*(time-self.last_time))*(1-alpha)+accel_angle*alpha
                self.last_time+=dtime
            else:
                raise Exception('please give time or dtime parameter')
        else:
            attitude=0
            self.last_time=time
        self.last_attitude=attitude
        return attitude
















