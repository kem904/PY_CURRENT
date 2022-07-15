import time
import serial
import sys
import glob

Arm_port = 'COM5'
Arm_baudrate = 9600
Arm_parity = 'N'


def angleToArm(angle):
    if angle > 90 or angle < -90:
        print('角度转换出错')
        return 0
    res = 2000 * (90.0 + angle) / 180.0 + 500.0  # 把舵机处于90度位置（1500）定义为0度 舵机运动范围为 2000-1000
    return int(res)  # 角度换算公式(2000 * 角度 / 180 + 500)


def send_str(port, baudrate, parity, value):                        #send byte through serial
    port_set = serial.Serial(port=port, baudrate=baudrate, parity=parity)
    port_set.write(value)
    byte_number_1 = 0
    byte_number_2 = 1
    while byte_number_1 != byte_number_2:
        byte_number_1 = port_set.inWaiting()
        time.sleep(0.1)
        byte_number_2 = port_set.inWaiting()
    receive_frame = port_set.read_all()
    return receive_frame


def wirte_angle(angle6,angle5,angle4,angle3):
    numbuffer = []
    numbuffer.append(0x55)  # 机械臂串口指令帧头
    numbuffer.append(0x55)
    numbuffer.append(17)  # 数据长度是十七个
    numbuffer.append(3)  # CMD_MULT_SERVO_MOVE 指令值 3
    numbuffer.append(4)  # 控制舵机个数
    # 舵机运动的时间
    temp=(1000).to_bytes(2,'big')
    numbuffer.append(temp[1])  # 舵机运动时间低八位
    numbuffer.append(temp[0])  # 舵机运动时间高八位
    # 舵机3参数
    temp=(angleToArm(angle3).to_bytes(2,'big'))
    numbuffer.append(3)  # 舵机id
    numbuffer.append(temp[1])     # 低八位
    numbuffer.append(temp[0])    # 高八位
    # 舵机4参数
    temp=(angleToArm(angle4).to_bytes(2,'big'))
    numbuffer.append(4)  # 舵机id
    numbuffer.append(temp[1])  # 低八位
    numbuffer.append(temp[0])  # 高八位
    # 舵机5参数
    temp=(angleToArm(angle5).to_bytes(2,'big'))
    numbuffer.append(5)  # 舵机id
    numbuffer.append(temp[1])  # 低八位
    numbuffer.append(temp[0])  # 高八位
    #舵机6参数
    temp = (angleToArm(angle6).to_bytes(2, 'big'))
    numbuffer.append(6)  # 舵机id
    numbuffer.append(temp[1])  # 低八位
    numbuffer.append(temp[0])  # 高八位
    a=bytearray(numbuffer)
    send_str(Arm_port,Arm_baudrate,Arm_parity,a)


def angle_reset():
    numbuffer = []
    numbuffer.append(0x55)  # 机械臂串口指令帧头
    numbuffer.append(0x55)
    numbuffer.append(24)  # 数据长度是24个
    numbuffer.append(3)  # CMD_MULT_SERVO_MOVE 指令值 3
    numbuffer.append(6)  # 控制舵机个数
    # 舵机运动的时间
    temp = (1000).to_bytes(2, 'big')
    numbuffer.append(temp[1])  # 舵机运动时间低八位
    numbuffer.append(temp[0])  # 舵机运动时间高八位
    i=1
    while i<=6:            # 舵机参数
        temp = (angleToArm(0).to_bytes(2, 'big'))
        numbuffer.append(i)  # 舵机id
        numbuffer.append(temp[1])  # 低八位
        numbuffer.append(temp[0])  # 高八位
        i=i+1
    a = bytearray(numbuffer)
    send_str(Arm_port, Arm_baudrate, Arm_parity, a)


if __name__ == '__main__':
    wirte_angle(30,-45,-45,-45)


