import pyb
import os
import math
from machine import I2C,SoftI2C
import time
import mpu6050
import as5048a
import algorithm
import pca9685
import pca9685_servo
import micropython
from micropython import const
import reciever
import servo_angle
import gc

micropython.opt_level(3)


#iic接口及iic设备初始化
iic1=SoftI2C(scl='X9',sda='X10',freq=150000)
mpu=mpu6050.MPU6050(iic1)
pca=pca9685.PCA9685(iic1)
servos=pca9685_servo.Servos(pca)
pca.duty(4,200)


#SPI及AS5048A初始化
spi1=pyb.SPI(1)
spi1.init(mode=pyb.SPI.MASTER,prescaler=8,bits=8)
as5048a_cs=pyb.Pin('X4',pyb.Pin.OUT_PP)
as5048a_cs.high()




#接收机UART及解码器初始化
reciever_UART=pyb.UART(3)
reciever_UART.init(115200)
recv_decoder=reciever.Reciever_decoder()


#时基定时器初始化
tick_timer=pyb.Timer(4,freq=80)


#算法对象初始化
USE_LEG_ROLL_PID=True
pitch_pid=algorithm.pid(-0.45,0,1.5,pid_value=120,value_max=180,value_min=0)
tail_servo_pid=algorithm.pid(-0.2,0,0,90,180,0,use_inte=True)
pitch_comp=algorithm.ComplementaryFilter()
if USE_LEG_ROLL_PID: #leg angle here is uniformed to 0 to 1 for convenent adjusting
    roll_pid=algorithm.pid(0.01,0,0,pid_value=0.5,value_max=1,value_min=0)
    leg_servo_pid=algorithm.pid(-0.1,0,0,0.5,1,0,use_inte=True)
    roll_comp=algorithm.ComplementaryFilter()


time.sleep(5)


#常量定义
#L_WINGSPAN_SERVO_PORT=const(0)
#R_WINGSPAN_SERVO_PORT=const(1)
#L_LEGYAW_SERVO_PORT=const(2)
#L_LEGPITCH_SERVO_PORT=const(4)
#SWING_SERVO_PORT=const(5)
#R_LEGYAW_SERVO_PORT=const(6)
#R_LEGYAW_SERVO_0POS=const(65)
#R_LEGPITCH_SERVO_PORT=const(8)
#TAIL_SERVO_PORT=const(10)
#ESC_PWM_PORT=const(12)

L_LEGYAW_SERVO_PORT=const(0)
L_LEGPITCH_SERVO_PORT=const(1)
R_LEGYAW_SERVO_PORT=const(2)
R_LEGPITCH_SERVO_PORT=const(3)
TAIL_SERVO_PORT=const(6)
ESC_PWM_PORT=const(7)
L_WINGSPAN_SERVO_PORT=const(8)
R_WINGSPAN_SERVO_PORT=const(9)
SWING_SERVO_PORT=const(5)


PITCH_INPUT_CHANNEL=const(1)   #**注意：此处定义的通道数值要比遥控器显示的小1**
YAW_INPUT_CHANNEL=const(0)
ROLL_INPUT_CHANNEL=const(3)
THROTTLE_INPUT_CHANNEL=const(2)
BREAK_INPUT_CHANNEL=const(7)
ESTOP_INPUT_CHANNEL=const(8)
L_LEGPITCH_INPUT_CHANNEL=const(5)
R_LEGPITCH_INPUT_CHANNEL=const(6)
LEG_SYNC_INPUT_CHANNEL=const(4)
WSPAN_CTRL1_INPUT_CHANNEL=const(9)
WSPAN_CTRL2_INPUT_CHANNEL=const(10)
WSPAN_SEL_INPUT_CHANNEL=const(11)
CHANNEL_RANGE=const(1342)  #遥控器通道行程
#变量定义
pitch=0
roll=0
recv_channels=[0]*16
mpu_data='N/A'

#主时基中断
@micropython.native
def tick(tim):
    global mpu_data,pitch,logfile,wing_angle_0,recv_channels,wing_span_mode,roll
    #t=time.ticks_ms()#test
    #获取输入
    mpu_data=mpu.get_values()
    recv_nbytes=reciever_UART.any()
    if recv_nbytes!=0:
        recv_data=reciever_UART.read(recv_nbytes)
        for i in recv_data:
            recv_decoder.passin(i)
    recv_channels=recv_decoder.get_channels_data()

    #计算和控制
    if recv_channels[ESTOP_INPUT_CHANNEL]>CHANNEL_RANGE*0.7:
        #翅膀收展及挥摆控制
        #print(wing_angle)#test
        if wing_angle>20:#折展状态切换
            if -20<recv_channels[WSPAN_SEL_INPUT_CHANNEL]-CHANNEL_RANGE/2<20:
                wing_span_mode=0 #自动折展模式
            elif recv_channels[WSPAN_SEL_INPUT_CHANNEL]<20:
                wing_span_mode=1 #受WSPAN_CTRL1_INPUT_CHANNEL通道控制
            elif recv_channels[WSPAN_SEL_INPUT_CHANNEL]>CHANNEL_RANGE-20:
                wing_span_mode=2 #受WSPAN_CTRL2_INPUT_CHANNEL通道控制
        if wing_span_mode==0:
            wingSpan_angle_base=servo_angle.calc_wingSpanAngle(wing_angle)
        elif wing_span_mode==1:
            wingSpan_angle_base=0+(recv_channels[WSPAN_CTRL1_INPUT_CHANNEL]/CHANNEL_RANGE*180)
        else:
            wingSpan_angle_base=0+(recv_channels[WSPAN_CTRL2_INPUT_CHANNEL]/CHANNEL_RANGE*180)

            
        LwingSpan_angle=wingSpan_angle_base+(recv_channels[ROLL_INPUT_CHANNEL]-CHANNEL_RANGE/2)/CHANNEL_RANGE*30
        RwingSpan_angle_N=wingSpan_angle_base-(recv_channels[ROLL_INPUT_CHANNEL]-CHANNEL_RANGE/2)/CHANNEL_RANGE*30
        if LwingSpan_angle<50:LwingSpan_angle=50  #折展限位
        if LwingSpan_angle>170:LwingSpan_angle=170
        if RwingSpan_angle_N<60:RwingSpan_angle_N=60
        if RwingSpan_angle_N>150:RwingSpan_angle_N=150
        servos.position(L_WINGSPAN_SERVO_PORT,degrees=LwingSpan_angle)
        servos.position(R_WINGSPAN_SERVO_PORT,degrees=180-RwingSpan_angle_N)
        #swing_angle=servo_angle.calc_swingAngle(wing_angle)
        #if swing_angle<60:swing_angle=60
        #if swing_angle>150:swing_angle=150
        #servos.position(SWING_SERVO_PORT,degrees=swing_angle)
        #腿部控制
        L_leg_baseangle=50;L_leg_minangle=45;L_leg_maxangle=85       # 调整
        R_leg_baseangle=55;R_leg_minangle=80;R_leg_maxangle=120
    
        if not recv_channels[BREAK_INPUT_CHANNEL]>CHANNEL_RANGE*0.7:#正常状态
            if USE_LEG_ROLL_PID:
                try:
                    roll_accel_angle=math.degrees(math.atan(mpu_data['AcX']/mpu_data['AcZ']))
                    roll=roll_comp.filt(0.00003052*mpu_data['GyY'],
                                        roll_accel_angle,
                                        0.5 if mpu_data['AcZ']>200 else 0,
                                        time=time.ticks_ms())
                except:
                    pass
                leg_Uangle_goal=roll_pid.calc(time.ticks_ms(),roll,
                                              -recv_channels[YAW_INPUT_CHANNEL]/CHANNEL_RANGE*120+45)
                leg_Uangle=leg_servo_pid.calc(time.ticks_ms(),leg_servo_pid.pid_value,leg_Uangle_goal)
                L_leg_yaw=L_leg_baseangle+leg_Uangle*60
                R_leg_yaw=R_leg_baseangle+leg_Uangle*60
            else:
                L_leg_yaw=L_leg_baseangle+recv_channels[YAW_INPUT_CHANNEL]/CHANNEL_RANGE*60
                R_leg_yaw=R_leg_baseangle+recv_channels[YAW_INPUT_CHANNEL]/CHANNEL_RANGE*60
            if L_leg_yaw>L_leg_maxangle:L_leg_yaw=L_leg_maxangle
            if L_leg_yaw<L_leg_minangle:L_leg_yaw=L_leg_minangle
            if R_leg_yaw>R_leg_maxangle:R_leg_yaw=R_leg_maxangle
            if R_leg_yaw<R_leg_minangle:R_leg_yaw=R_leg_minangle
            servos.position(L_LEGYAW_SERVO_PORT,degrees=L_leg_yaw)
            servos.position(R_LEGYAW_SERVO_PORT,degrees=R_leg_yaw)
        else:                #刹车状态
            servos.position(L_LEGYAW_SERVO_PORT,degrees=30)
            servos.position(R_LEGYAW_SERVO_PORT,degrees=150)
        servos.position(L_LEGPITCH_SERVO_PORT,degrees=25*(recv_channels[L_LEGPITCH_INPUT_CHANNEL]/CHANNEL_RANGE)+75)
        if recv_channels[LEG_SYNC_INPUT_CHANNEL]>CHANNEL_RANGE*0.7:
            servos.position(R_LEGPITCH_SERVO_PORT,degrees=-25*(recv_channels[L_LEGPITCH_INPUT_CHANNEL]/CHANNEL_RANGE)+105)#同步模式
        else:
            servos.position(R_LEGPITCH_SERVO_PORT,degrees=-25*(recv_channels[R_LEGPITCH_INPUT_CHANNEL]/CHANNEL_RANGE)+105)#非同步模式
        #俯仰角控制
        try:
            pitch_accel_angle=math.degrees(math.atan(mpu_data['AcY']/mpu_data['AcZ']))
            pitch=pitch_comp.filt(-0.00003052*mpu_data['GyX'],
                                  pitch_accel_angle,
                                  0.5 if mpu_data['AcZ']>200 else 0,
                                  time=time.ticks_ms())
            #print(pitch)#test
        except:
            pass
        tail_angle_goal=pitch_pid.calc(time.ticks_ms(),pitch,
                                       recv_channels[PITCH_INPUT_CHANNEL]/CHANNEL_RANGE*90-45)
        if tail_angle_goal<30:
            tail_angle_goal=30
        tail_angle=tail_servo_pid.calc(time.ticks_ms(),tail_servo_pid.pid_value,tail_angle_goal)
        if USE_LEG_ROLL_PID:   #腿部闭环模式下动腿垂尾巴
            tail_angle-=abs(leg_Uangle-0.5)*20
        servos.position(TAIL_SERVO_PORT,degrees=tail_angle)
        #油门控制
        pca.duty(ESC_PWM_PORT,recv_channels[THROTTLE_INPUT_CHANNEL]/CHANNEL_RANGE*(120)+225)
    else:
        for i in [0,1,2,3,4,5,6,8,9,10,11]:
            servos.position(i,degrees=90)
        pca.duty(ESC_PWM_PORT,225)
        wing_angle_0+=wing_angle
        if wing_angle_0>300:
            wing_angle_sensor.offset=180
            wing_angle_0-=180

    #gc.collect()
    #print(time.ticks_ms()-t) #test





#定义定时器中断
def tick_timer_callback(tim):
    micropython.schedule(tick,tim)

micropython.alloc_emergency_exception_buf(50)
tick_timer.callback(tick_timer_callback)


#主循环
while(True):
    if LOG:
        logfile.flush()
        time.sleep(2)
    




