#!/usr/bin/env python
import json

import numpy as np
import time
import sys
import rospy
from std_msgs.msg import String

FR_HOST = "192.168.1.10"

FeedBackType_2k1 = np.dtype([

    ('frame_test', np.uint16),  # 帧头
    ('frame_count', np.uint8,),  # 帧计数
    ('frame_length', np.uint16),  # 数据长度

    ('program_state', np.uint8),  # 程序运行状态
    ('robot_state', np.uint8,),  # 机器人运动状态
    ('error_code', np.uint8),  # 故障码
    ('robot_mode', np.uint8),  # 机器人模式

    ('jt_cur_pos', np.float64, (6,)),  # 1-6轴当前关节位置
    ('tl_cur_pos', np.float64, (6,)),  # 工具当前             位置xyx 姿态abc
    ('flange_cur_pos', np.float64, (6,)),  # 末端法兰当前          位置xyz 姿态abc
    ('jt_cur_tor', np.float64, (6,)),  # 1-6轴当前扭矩

    ('toolNum', np.int32),  # 工具号
    ('workPieceNum', np.int32),  # 工件号

    ('cl_dgt_output', np.uint8, (2,)),  # 控制箱数字量IO输出    [H 15-8 | L 7-0]
    ('tl_dgt_output_l', np.uint8),  # 工具数字量IO输出 仅bit0-bit1有效
    ('cl_dgt_input', np.uint8, (2,)),  # 控制箱数字量IO输入    [15-8]
    ('tl_dgt_input_l', np.uint8),  # 工具数字量IO输入 仅bit0-bit1有效

    ('motion_done', np.int32),  # 运动到位信号
    ('frame_check_sum', np.uint16),  # 和校验
])

FeedBackType_2k4 = np.dtype([
    ('frame_test', np.uint16),  # 帧头
    ('frame_count', np.uint8,),  # 帧计数
    ('frame_length', np.uint16),  # 数据长度

    ('program_state', np.uint8),  # 程序运行状态
    ('robot_state', np.uint8,),  # 机器人运动状态
    ('main_code', np.int32),  # 主故障码
    ('sub_code', np.int32),  # 子故障码
    ('robot_mode', np.uint8),  # 机器人模式

    ('jt_cur_pos', np.float64, (6,)),  # 1-6轴当前关节位置
    ('tl_cur_pos', np.float64, (6,)),  # 工具当前             位置xyx 姿态abc
    ('flange_cur_pos', np.float64, (6,)),  # 末端法兰当前          位置xyz 姿态abc
    ('actual_qd', np.float64, (6,)),  # 当前关节1-6速度
    ('actual_qdd', np.float64, (6,)),  # 当前关节1-6加速度

    ('target_TCP_CmpSpeed', np.float64, (2,)),  # TCP合成指令速度       [位置 | 姿态]
    ('target_TCP_Speed', np.float64, (6,)),  # TCP指令速度           xyz rxyz
    ('actual_TCP_CmpSpeed', np.float64, (2,)),  # TCP合成实际速度       [位置 | 姿态]
    ('actual_TCP_Speed', np.float64, (6,)),  # TCP实际速度           xyz rzyx

    ('jt_cur_tor', np.float64, (6,)),  # 1-6轴当前扭矩

    ('toolNum', np.int32),  # 工具号
    ('workPieceNum', np.int32),  # 工件号

    ('cl_dgt_output', np.uint8, (2,)),  # 控制箱数字量IO输出    [H 15-8 | L 7-0]
    ('tl_dgt_output_l', np.uint8),  # 工具数字量IO输出      [7-0]
    ('cl_dgt_input', np.uint8, (2,)),  # 控制箱数字量IO输入    [15-8]
    ('tl_dgt_input_l', np.uint8),  # 工具数字量IO输入      [7-0]
    ('cl_analog_input', np.uint16, (2,)),  # 控制箱模拟量输入      [0-1]
    ('tl_analog_input', np.uint16),  # 工具模拟量输入

    ('ft_sensor_raw_data', np.float64, (6,)),  # 力矩传感器原始数据    Fxyx Txyz
    ('ft_sensor_data', np.float64, (6,)),  # 力矩传感器数据        Fxyx Txyz
    ('ft_sensor_active', np.uint8,),  # 力矩传感器激活状态

    ('EmergencyStop', np.uint8,),  # 急停标志
    ('motion_done', np.int32),  # 运动到位信号
    ('gripper_motiondone', np.uint8,),  # 夹爪运动完成信号

    ('mc_queue_len', np.int32),  # 运动指令队列长度
    ('collisionState', np.uint8,),  # 碰撞检测
    ('trajectory_pnum', np.int32),  # 轨迹点编号

    ('checksum', np.uint16,)
])

rospy.init_node('robot_state_publisher')
topic_name = "/arm_status"
publisher = rospy.Publisher(topic_name, String, queue_size=1)


def get_from_title(field_name, a_data):
    try:
        a = a_data[field_name]
        if a.size > 1:
            res = a.tolist()
        else:
            res = a.item()
    except:
        return None
    return res


def getStatus():
    import socket
    # 20004
    # 20001
    # 8083 # 维护模式 frrts2021 机器人状态采样周期 10ms
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    STATUS_PORT = 20001
    try:
        s.connect((FR_HOST, STATUS_PORT))
        s.settimeout(None)
    except socket.timeout:
        rospy.logerr(f"Connect to arm status port {STATUS_PORT} timeout, exiting...")
        rospy.logerr(f"You may need to restart the robot arm...")
        exit(1)
    rospy.loginfo(f"Launch arm status node...")
    idx = 0
    while not rospy.is_shutdown():
        all = s.recv(221)  # 100ms?
        data = all[0:221]
        try:
            a = np.frombuffer(data, dtype=FeedBackType_2k1)
        except:
            continue
        d = {}
        for field_name in FeedBackType_2k1.names:
            d[field_name] = get_from_title(field_name, a)
        json_data = json.dumps(d)
        publisher.publish(json_data)
        idx += 1

while not rospy.is_shutdown():
    getStatus()
