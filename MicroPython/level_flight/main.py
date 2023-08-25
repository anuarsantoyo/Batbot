import pyb
import os
import math
from machine import I2C, SoftI2C
import time
import mpu6050
import as5048a
import pid
import pca9685
import pca9685_servo
import micropython
from micropython import const
import reciever
import servo_angle

#micropython.opt_level(3)
LOG=False

#iic接口及iic设备初始化
iic1=SoftI2C(scl='X9',sda='X10',freq=150000)
mpu=mpu6050.MPU6050(iic1)
'''
pca=pca9685.PCA9685(iic1)
servos=pca9685_servo.Servos(pca)
pca.duty(12,220)
for i in [0,2,4,6,8,10]:
    servos.position(i,degrees=90)
'''
#舵机电调初始化
Lleg_servo=pyb.Servo(1)
Rleg_servo=pyb.Servo(2)
Wspan_servo=pyb.Servo(3)
ESCon=pyb.Servo(4)
ESCon.pulse_width(1000) #1200启动


#SPI及AS5048A初始化
spi2=pyb.SPI(2)
spi2.init(mode=pyb.SPI.MASTER,prescaler=8,bits=8)
as5048a_cs=pyb.Pin('Y4',pyb.Pin.OUT_PP)
as5048a_cs.high()
wing_angle_sensor=as5048a.AS5048A(spi2,as5048a_cs)
wing_angle_0=wing_angle_sensor.read_angle()
if wing_angle_0>300:
    wing_angle_sensor.offset=180
    wing_angle_0-=180



#接收机UART及解码器初始化
reciever_UART=pyb.UART(3)
reciever_UART.init(115200)
recv_decoder=reciever.Reciever_decoder()


#时基定时器初始化
tick_timer=pyb.Timer(4,freq=80)


#pid对象初始化
pitch_pid=pid.pid(0.7,0,0,pid_value=20,value_max=60,value_min=-60)


#创建并打开记录文件
if LOG:
    log_dir = './log'
    files = os.listdir(log_dir)
    file_nums = [int(file.split('_')[1].split('.')[0]) for file in files if file.startswith('Datalog_')]
    max_file_num = sorted(file_nums)[-1] if file_nums else 0
    new_file_name = "Datalog_%d.txt"%(max_file_num+1)
    log_file_path = log_dir+'/'+new_file_name
    logfile=open(log_file_path,'w')
    del log_dir,files,file_nums,max_file_num,new_file_name

#exit
#等待电调初始化完成
time.sleep(5)


#常量定义
'''
WINGSPAN_SERVO_PORT=const(0)
L_LEGYAW_SERVO_PORT=const(2)
L_LEGPITCH_SERVO_PORT=const(4)
R_LEGYAW_SERVO_PORT=const(6)
R_LEGYAW_SERVO_0POS=const(65)
R_LEGPITCH_SERVO_PORT=const(8)
TAIL_SERVO_PORT=const(10)
ESC_PWM_PORT=const(12)
'''

PITCH_INPUT_CHANNEL=const(1)   #**注意：此处定义的通道数值要比遥控器显示的小1**
ROLL_INPUT_CHANNEL=const(3)
YAW_INPUT_CHANNEL=const(0)
THROTTLE_INPUT_CHANNEL=const(2)
BREAK_INPUT_CHANNEL=const(7)
ESTOP_INPUT_CHANNEL=const(8)
L_LEGPITCH_INPUT_CHANNEL=const(5)
R_LEGPITCH_INPUT_CHANNEL=const(6)
LEG_SYNC_INPUT_CHANNEL=const(4)
CHANNEL_RANGE=const(1342)  #遥控器通道行程
#变量定义
wing_angle=0
pitch=0
pitch_list=[0]*10
recv_channels=[0]*16
mpu_data='N/A'
#主时基中断
def tick(tim):
    global wing_angle,mpu_data,pitch,pitch_list,logfile,wing_angle_0,recv_channels
    #t=time.ticks_ms()
    #获取输入
    mpu_data=mpu.get_values()
    wing_angle=(wing_angle_sensor.read_angle()-wing_angle_0)
    recv_nbytes=reciever_UART.any()
    if recv_nbytes!=0:
        recv_data=reciever_UART.read(recv_nbytes)
        for i in recv_data:
            recv_decoder.passin(i)
    recv_channels=recv_decoder.get_channels_data()


    #数据记录
    if LOG:
        logfile.write('%07d~wAng%5.1f~mpu:AcX%6dAcY%6dAcZ%6dGyX%5dGyY%5dGyZ%5d~recv:Thr%4dPch%4dYaw%4dBrk%4dEst%4dLlg%4dRlg%4dSlg%4d\n'%\
                      (time.ticks_ms(),wing_angle,mpu_data['AcX'],\
                       mpu_data['AcY'],mpu_data['AcZ'],mpu_data['GyX'],\
                       mpu_data['GyY'],mpu_data['GyZ'],recv_channels[THROTTLE_INPUT_CHANNEL],\
                       recv_channels[PITCH_INPUT_CHANNEL],recv_channels[YAW_INPUT_CHANNEL],\
                       recv_channels[BREAK_INPUT_CHANNEL],recv_channels[ESTOP_INPUT_CHANNEL],\
                       recv_channels[L_LEGPITCH_INPUT_CHANNEL],recv_channels[R_LEGPITCH_INPUT_CHANNEL],\
                       recv_channels[LEG_SYNC_INPUT_CHANNEL]))


    #计算和控制
    if recv_channels[ESTOP_INPUT_CHANNEL]>CHANNEL_RANGE*0.7:
        #翅膀展开幅度控制
        #print(wing_angle)#test
        wingSpan_angle=servo_angle.calc_wingSpanAngle(wing_angle)
        if wingSpan_angle<-70:
            wingSpan_angle=-70
        if wingSpan_angle>10:
            wingSpan_angle=10
        #print(wing_angle,wingSpan_angle)
        Wspan_servo.angle(wingSpan_angle)

        #腿部舵机控制（包含俯仰角pid）
        try:
            pitch=math.degrees(math.atan(mpu_data['AcX']/mpu_data['AcZ']))
        except:
            pass
        pitch_list.append(pitch)
        del(pitch_list[0])
        leg_baseAngle=pitch_pid.calc(sum(pitch_list)/len(pitch_list),recv_channels[PITCH_INPUT_CHANNEL]/CHANNEL_RANGE*90-45)
        leg_baseAngle+=wing_angle-15
        Lleg_angle=leg_baseAngle+(recv_channels[ROLL_INPUT_CHANNEL]-CHANNEL_RANGE/2)/CHANNEL_RANGE*30
        Rleg_angle_N=leg_baseAngle-(recv_channels[ROLL_INPUT_CHANNEL]-CHANNEL_RANGE/2)/CHANNEL_RANGE*30
        if Lleg_angle<-20:Lleg_angle=-20
        if Lleg_angle>60:Lleg_angle=60
        if Rleg_angle_N<-20:Rleg_angle_N=-20
        if Rleg_angle_N>60:Rleg_angle_N=60
        Lleg_servo.angle(Lleg_angle)
        Rleg_servo.angle(-Rleg_angle_N)

        
        #油门控制
        thr=int(1190+500*recv_channels[THROTTLE_INPUT_CHANNEL]/CHANNEL_RANGE)
        if (servo_angle.stroke==0 and wing_angle>5)\
           or (servo_angle.stroke==1 and wing_angle>30):
            thr-=int(400*recv_channels[THROTTLE_INPUT_CHANNEL]/CHANNEL_RANGE)
        ESCon.pulse_width(thr)
    else:
        ESCon.pulse_width(1100)
        wing_angle_0+=wing_angle
        if wing_angle_0>300:
            wing_angle_sensor.offset=180
            wing_angle_0-=180


    #print(time.ticks_ms()-t)





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
    




