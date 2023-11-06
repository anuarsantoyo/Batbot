import pyb
import os
import math
from machine import I2C,SoftI2C
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

micropython.opt_level(3)
LOG=False

#iic接口及iic设备初始化
iic1=SoftI2C(scl='X9',sda='X10',freq=150000)
mpu=mpu6050.MPU6050(iic1)
pca=pca9685.PCA9685(iic1)
servos=pca9685_servo.Servos(pca)
pca.duty(12,220)
for i in [0,2,4,6,8,10]:
    servos.position(i,degrees=90)


#SPI及AS5048A初始化
spi1=pyb.SPI(1)
spi1.init(mode=pyb.SPI.MASTER,prescaler=8,bits=8)
as5048a_cs=pyb.Pin('X4',pyb.Pin.OUT_PP)
as5048a_cs.high()
wing_angle_sensor=as5048a.AS5048A(spi1,as5048a_cs)
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
pitch_pid=pid.pid(0.5,0,0,pid_value=65,value_max=180,value_min=0)


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
WINGSPAN_SERVO_PORT=const(0)
L_LEGYAW_SERVO_PORT=const(2)
L_LEGPITCH_SERVO_PORT=const(4)
R_LEGYAW_SERVO_PORT=const(6)
R_LEGYAW_SERVO_0POS=const(65)
R_LEGPITCH_SERVO_PORT=const(8)
TAIL_SERVO_PORT=const(10)
ESC_PWM_PORT=const(12)

PITCH_INPUT_CHANNEL=const(1)   #**注意：此处定义的通道数值要比遥控器显示的小1**
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
recv_channels=[0]*16
mpu_data='N/A'
#主时基中断
def tick(tim):
    global wing_angle,mpu_data,pitch,logfile,wing_angle_0,recv_channels
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
        if wingSpan_angle<60:
            wingSpan_angle=60
        if wingSpan_angle>150:
            wingSpan_angle=150
        servos.position(WINGSPAN_SERVO_PORT,degrees=wingSpan_angle)
        #俯仰角控制
        try:
            pitch=math.degrees(math.asin(mpu_data['AcX']/mpu_data['AcZ']))
        except:
            pass
        tail_angle=pitch_pid.calc(pitch,recv_channels[PITCH_INPUT_CHANNEL]/CHANNEL_RANGE*90-45)
        if tail_angle<30:
            tail_angle=30
        servos.position(TAIL_SERVO_PORT,degrees=tail_angle)
        #腿部控制
        if not recv_channels[BREAK_INPUT_CHANNEL]>CHANNEL_RANGE*0.7:#正常状态
            L_leg_yaw=60+recv_channels[YAW_INPUT_CHANNEL]/CHANNEL_RANGE*60
            R_leg_yaw=60+recv_channels[YAW_INPUT_CHANNEL]/CHANNEL_RANGE*60
            if L_leg_yaw>100:L_leg_yaw=100
            if L_leg_yaw<60:L_leg_yaw=60
            if R_leg_yaw>120:R_leg_yaw=120
            if R_leg_yaw<80:R_leg_yaw=80
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
        #油门控制
        pca.duty(ESC_PWM_PORT,recv_channels[THROTTLE_INPUT_CHANNEL]/CHANNEL_RANGE*(80)+231)
    else:
        for i in [0,2,4,6,8,10]:
            pca.duty(i,0)
        pca.duty(ESC_PWM_PORT,220)
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
    logfile.flush()
    time.sleep(2)
    




