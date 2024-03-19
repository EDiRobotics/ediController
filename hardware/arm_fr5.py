import os
import pdb
import sys
import threading
import time
from ctypes import cdll
from typing import Dict, List, Tuple
import numpy as np
from xmlrpc.client import ProtocolError

# Get the directory containing this script
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
path = os.path.join(current_dir, "frrpc.so")
if not os.path.exists(path):
    print("Please put 'frrpc.so' into the file directory.")
    raise FileNotFoundError
frrpc = cdll.LoadLibrary(path)
import frrpc


class FR5:
    """
    Defining the workspace limit
    """
    """ 
    The workspace should be:
    x_min, x_max = 300, 800
    y_min, y_max = -300, 60
    z_min, z_max = 140, 350
    """
    x_min, x_max = -300, 800
    y_min, y_max = -900, 600
    z_min, z_max = 140, 1000

    _filter_width = 6

    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.robot = frrpc.RPC(self.robot_ip)
        # self.robot = ThreadSafeProxy(self.robot)
        print(f'\033[37m[__init__]: Robot initializing.. \033[0m')
        # gripper 14 cm
        gripper_length = 13.0
        # TODO: default_pose not as expected
        default_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        gripper_pose = [0.0, 0.0, float(gripper_length), 0.0, 0.0, 0.0]
        self.robot.SetToolCoord(0, default_pose, 0, 0)
        self.robot.SetToolCoord(1, gripper_pose, 0, 0)
        self.move_end([600, 0, 300, -180, 0, 90])
        self.close_gripper()
        joint_pos = self.robot.GetActualJointPosRadian(0)
        # joint_pos = self.robot.GetActualJointPosDegree(0)
        joint_pos = np.array(joint_pos).reshape(1, 6)
        self._last_pos = np.tile(joint_pos, (self._filter_width, 1))
        self._last_vel = np.zeros(self._filter_width, 6)
        print(f'\033[37m[__init__]: Robot initialized. \033[0m')

    def move(self, action, time_t):
        self.set_gripper(action[-1])
        self.move_servo(action[:6], time_t)

    def move_servo(self, joint, time_t):
        if len(joint) != 6:
            print(f"[move_joint_servo] joint is {joint} which has invalid length")
            return 3
        loc = self.robot.GetForwardKin(joint)[1:]
        x, y, z = loc[0], loc[1], loc[2]
        if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max and self.z_min <= z <= self.z_max):
            print(f"[move_joint_servo] Out of workspace! joint \n{joint}, loc \n{loc}, force return.")
            print(
                f"Workspace limits: X({self.x_min}, {self.x_max}), Y({self.y_min}, {self.y_max}), Z({self.z_min}, {self.z_max})")
            return 14
        joint = self._ema_joint(joint, time_t)
        try:
            ret = self.robot.ServoJ(joint, 0.0, 0.0, time_t, 0.0, 0.0)
            return ret
        except ProtocolError as pe:
            # return -3
            return 0
        except Exception as e:
            print(f"[move_joint_servo] An error occurs: ", e)
            return 14

    def reconnect(self):
        self.robot = frrpc.RPC(self.robot_ip)

    def _gripper_ctr(self, ef_joint):
        try:
            gripper_pos = ef_joint
            return self.robot.SetToolAO(0, float(gripper_pos), 1)
        except Exception as e:
            print(f"[_gripper_ctr] An error occurs:", e)

    def open_gripper(self):
        return self._gripper_ctr(800)

    def close_gripper(self):
        return self._gripper_ctr(100)

    def set_gripper(self, p):
        if 0 <= p <= 1000:
            return self._gripper_ctr(p)
        else:
            print(f"[set_gripper] Gripper control angle {p} is not valid")
            return 4

    def move_end(self, pose):
        pose = [float(x) for x in pose]
        if pose[2] < 180:
            print(f"[move_end] Robot z value {pose[2]} is dangerous!")
            pose[2] = 200.0
        return self.robot.MoveCart(pose, 1, 0, 100.0, 100.0, 100.0, -1.0, -1)

    def detect_errors(self) -> Tuple[int, List[Dict[int, str]]]:
        rets = self.robot.GetRobotErrorCode()
        rets += self.robot.GetRobotCurJointsConfig()
        errors = [{ret: robot_errors[str(ret)]} for ret in rets]
        e = 0 if all(ret == 0 for ret in rets) else 1
        return e, errors

    def lookup_error(self, ret):
        if ret is None:
            ret = 0
        return {ret: robot_errors[str(ret)]}

    def clear_errors(self):
        try:
            return self.robot.ResetAllError()
        except:
            return -1

    def _ema_joint(self, joint, time_t, gamma=0.2, clip_acc=2.0):
        current_vel = (np.array(joint) - self._last_pos[-1]) / time_t
        current_vel = gamma * current_vel + (1 - gamma) * self._last_vel[-1]
        current_acc = (current_vel - self._last_vel[-1]) / time_t

        if np.linalg.norm(current_acc) > clip_acc:
            current_acc = (current_acc / np.linalg.norm(current_acc)) * clip_acc
            current_vel = self._last_vel[-1] + current_acc * time_t

        updated_joint = self._last_pos[-1] + current_vel * time_t + 0.5 * current_acc * (time_t ** 2)

        self._last_vel = np.roll(self._last_vel, -1, axis=0)
        self._last_vel[-1] = current_vel
        self._last_pos = np.roll(self._last_pos, -1, axis=0)
        self._last_pos[-1] = updated_joint

        return updated_joint.tolist()


def fr5():
    try:
        robot = FR5('192.168.1.10')
    except:
        exit("Can not connect to robot, exiting!")
    return robot


robot_errors = {"-4": ("xmlrpc接口执行失败", "请联系售后工程"),
                "-3": ("xmlrpc通讯失败", "请检查网络连接以及服务器IP地址是否正确"),
                "-2": ("与控制器通讯异常", "检查与控制器软硬件连接"),
                "-1": ("其他错误", "联系售后工程师查看控制器日志"),
                "0": ("调用成功", ""),
                "3": ("接口参数个数不一致", "检查接口参数个数"),
                "4": ("接口参数值异常", "检查参数值类型或范围"),
                "8": ("轨迹文件打开失败", "检查TPD轨迹文件是否存在或轨迹名是否正确"),
                "9": ("TPD文件名发送失败", "检查 TPD 轨迹名是否正确"),
                "10": ("TPD文件内容发送失败", "检查 TPD 文件内容是否正确"),
                "14": ("接口执行失败", "检查web界面是否报故障或状态反馈是否报故障"),
                "15": ("TDP记录点数超限", "重新记录"),
                "18": ("机器人程序正在运行，请先停止", "先停止程序，再进行其他操作"),
                "25": ("数据异常，计算失败", "重新标定或辨识"),
                "28": ("逆运动学计算结果异常", "检查位姿是否合理"),
                "29": ("ServoJ关节超限", "检查关节数据是否在合理范围"),
                "30": ("不可复位故障，请断电重启控制箱", "请断电重启控制箱"),
                "32": ("关节超限", "切换至拖动模式，将关节移动至软限位范围"),
                "34": ("工件号错误", "请检查工件号是否合理"),
                "36": ("文件名过长", "请缩减文件名长度"),
                "37": ("工具号错误", "请检查工具号是否合理"),
                "38": ("奇异位姿，计算失败", "请更换位姿"),
                "40": ("速度百分比超限", "检查速度百分比是否合理"),
                "42": ("姿态变化过大", "插入中间姿态进行过度"),
                "59": ("力/扭矩传感器未激活", "激活力传感器"),
                "60": ("力/扭矩传感器参考坐标系未切换至工具", "将力传感器坐标系切换至工具"),
                "64": ("未加入指令队列", "联系售后工程师查看控制器日志"),
                "66": ("整圆/螺旋线指令中间点1错误", "检查中间点1数据是否正确"),
                "67": ("整圆/螺旋线指令中间点2错误", "检查中间点2数据是否正确"),
                "68": ("整圆/螺旋线指令中间点3错误", "检查中间点3数据是否正确"),
                "69": ("圆弧指令中间点错误", "检查中间点数据是否正确"),
                "70": ("圆弧指令目标点错误", "检查目标点数据是否正确"),
                "73": ("夹爪运动报错", "检查夹爪通信状态是否正常"),
                "74": ("直线指令点错误", "检查点位数据是否正确"),
                "75": ("通道错误", "检查IO编号是否在范围内"),
                "76": ("等待超时", "检查IO信号是否输入或接线是否正确"),
                "82": ("TPD指令点错误", "重新记录示教轨迹"),
                "83": ("TPD指令工具与当前工具不符", "更改为TPD示教时所用的工具坐标系"),
                "94": ("样条指令点错误", "检查点位数据是否正确"),
                "95": ("样条参数错误", "检查样条参数是否合理"),
                "108": ("螺旋线指令起点错误", "检查起点数据是否正确"),
                "112": ("给定位姿无法到达", "检查目标位姿是否合理")
                }

if __name__ == "__main__":
    robot = fr5()
