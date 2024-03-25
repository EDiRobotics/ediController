import os
import pdb
import sys
import threading
import time
import traceback
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


class RetryingWrapper:
    def __init__(self, robot, max_retries=30, retry_delay=0):
        self.robot = robot
        self.max_retries = max_retries
        self.retry_delay = retry_delay

    def __getattr__(self, name):
        def wrapped_method(*args, **kwargs):
            retries = 0
            while retries < self.max_retries:
                try:
                    method = getattr(self.robot, name)
                    return method(*args, **kwargs)
                except Exception as pe:
                    retries += 1
                    if retries == self.max_retries:
                        raise pe
                    if self.retry_delay > 0:
                        time.sleep(self.retry_delay)

        return wrapped_method


class FR5:
    """
    Defining the workspace limit
    """
    """ 
    The workspace should be:
    x_min, x_max = 300, 800
    y_min, y_max = -300, 60
    z_min, z_max = 150, 350
    """

    x_min, x_max = 250, 800
    y_min, y_max = -300, 60
    z_min, z_max = 130, 400

    gamma = 0.2
    clip_vel = np.array([30, 30, 30, 100, 50, 50])
    clip_acc = np.array([250, 250, 250, 400, 300, 800])

    safe_bound = 30
    x_min_soft, x_max_soft = x_min + safe_bound, x_max - safe_bound
    y_min_soft, y_max_soft = y_min + safe_bound, y_max - safe_bound
    z_min_soft, z_max_soft = z_min + safe_bound, z_max - safe_bound
    _filter_width = 200

    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.robot = RetryingWrapper(frrpc.RPC(self.robot_ip))
        self._last_pos = np.zeros((self._filter_width, 6))
        self._last_vel = np.zeros((self._filter_width, 6))
        self._last_acc = np.zeros((self._filter_width, 6))
        self._last_cart_pos = np.zeros((self._filter_width, 6))
        self._last_cart_pos_d = np.zeros((self._filter_width, 6))
        self._last_cart_vel = np.zeros((self._filter_width, 6))
        self._last_cart_vel_d = np.zeros((self._filter_width, 6))
        self._last_cart_acc = np.zeros((self._filter_width, 6))
        self._last_cart_acc_d = np.zeros((self._filter_width, 6))
        self.initialize()

    def initialize(self):
        print(f'\033[37m[__init__]: Robot initializing.. \033[0m')
        # gripper 14 cm
        gripper_length = 13.0
        # TODO: default_pose not as expected
        default_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        gripper_pose = [0.0, 0.0, float(gripper_length), 0.0, 0.0, 0.0]
        self.robot.ResetAllError()
        self.robot.SetToolCoord(0, default_pose, 0, 0)
        self.robot.SetToolCoord(1, gripper_pose, 0, 0)
        self.move_end([600, 0, 300, -180, 0, 90])
        self.close_gripper()
        joint_pos = self.robot.GetActualJointPosDegree(0)[1:]
        joint_pos = np.array(joint_pos).reshape(1, 6)
        self._last_pos = np.tile(joint_pos, (self._filter_width, 1))
        self._last_vel = np.zeros((self._filter_width, 6))
        self._last_acc = np.zeros((self._filter_width, 6))
        self._last_cart_pos = np.zeros((self._filter_width, 6))
        self._last_cart_pos_d = np.zeros((self._filter_width, 6))
        self._last_cart_vel = np.zeros((self._filter_width, 6))
        self._last_cart_vel_d = np.zeros((self._filter_width, 6))
        self._last_cart_acc = np.zeros((self._filter_width, 6))
        self._last_cart_acc_d = np.zeros((self._filter_width, 6))
        print(f'\033[37m[__init__]: Robot initialized. \033[0m')

    def move(self, action, time_t):
        self.set_gripper(action[-1])
        return self.move_servo(action[:6], time_t)

    def move_servo(self, joint, time_t):
        try:
            # if self.robot.ResetAllError() != 0:
            #     pdb.set_trace()
            #     print(f"[move_joint._servo] Cannot clear errors, need to restart")
            #     return -1

            loc = self.robot.GetForwardKin(joint)[1:]
            desired_loc = [item for item in loc]
            x, y, z = loc[0], loc[1], loc[2]
            out_of_workspace = False
            if not (
                    self.x_min_soft <= x <= self.x_max_soft and self.y_min_soft <= y <= self.y_max_soft and self.z_min_soft <= z <= self.z_max_soft):
                desired_loc[0] = max(self.x_min_soft, min(self.x_max_soft, loc[0]))
                desired_loc[1] = max(self.y_min_soft, min(self.y_max_soft, loc[1]))
                desired_loc[2] = max(self.z_min_soft, min(self.z_max_soft, loc[2]))
                joint = self.robot.GetInverseKin(0, [float(i) for i in desired_loc], -1)[1:]
                out_of_workspace = True
            gamma = self.gamma if not out_of_workspace else 0.4
            # start_time = time.time()
            joint = self._average_joint(joint, time_t, gamma=gamma, clip_vel=self.clip_vel, clip_acc=self.clip_acc)
            # print(f"{time.time() - start_time}")
            # current_vel = (np.array(joint) - self._last_pos[-1]) / time_t
            # current_acc = (current_vel - self._last_vel[-1]) / time_t
            # self._last_acc = np.roll(self._last_acc, -1, axis=0)
            # self._last_acc[-1] = current_acc
            # self._last_vel = np.roll(self._last_vel, -1, axis=0)
            # self._last_vel[-1] = current_vel
            # self._last_pos = np.roll(self._last_pos, -1, axis=0)
            # self._last_pos[-1] = joint

            loc = self.robot.GetForwardKin(joint)[1:]
            x, y, z = loc[0], loc[1], loc[2]

            # self._last_cart_pos = np.roll(self._last_cart_pos, -1, axis=0)
            # self._last_cart_pos[-1] = loc
            # self._last_cart_vel = np.roll(self._last_cart_vel, -1, axis=0)
            # self._last_cart_vel[-1] = (self._last_cart_pos[-1] - self._last_cart_pos[-2]) / time_t
            # self._last_cart_acc = np.roll(self._last_cart_acc, -1, axis=0)
            # self._last_cart_acc[-1] = (self._last_cart_vel[-1] - self._last_cart_vel[-2]) / time_t
            #
            # self._last_cart_pos_d = np.roll(self._last_cart_pos_d, -1, axis=0)
            # self._last_cart_pos_d[-1] = desired_loc
            # self._last_cart_vel_d = np.roll(self._last_cart_vel_d, -1, axis=0)
            # self._last_cart_vel_d[-1] = (self._last_cart_pos_d[-1] - self._last_cart_pos[-2]) / time_t
            # self._last_cart_acc_d = np.roll(self._last_cart_acc_d, -1, axis=0)
            # self._last_cart_acc_d[-1] = (self._last_cart_vel_d[-1] - self._last_cart_vel[-2]) / time_t

            if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max and self.z_min <= z <= self.z_max):
                print(f"[move_joint_servo] Out of workspace! joint \n{joint}, loc \n{loc}.")
                # pdb.set_trace()
                self.initialize()
                return 0
            ret = self.robot.ServoJ(joint, 0.0, 0.0, time_t, 0.0, 0.0)
            return ret
        except Exception as e:
            print("_last_pos", self._last_pos)
            print("_last_vel", self._last_vel)
            traceback.print_exc()
            print(f"[move_joint_servo] An error occurs: ", e)
            return 0

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

    def _average_joint(self, joint, time_t, gamma=0.2, clip_vel=None, clip_acc=None, window_size=10):
        current_vel = (np.array(joint) - self._last_pos[-1]) / time_t
        weights = np.ones(window_size)
        weights /= weights.sum()
        # current_vel = gamma * current_vel + (1 - gamma) * self._last_vel[-1]
        current_vel = (1 - gamma) * np.dot(weights, self._last_vel[-1:-1 - window_size:-1]) + gamma * current_vel

        current_acc = (current_vel - self._last_vel[-1]) / time_t
        if clip_acc is not None:
            current_acc = np.clip(current_acc, -clip_acc, clip_acc)

        current_vel = self._last_vel[-1] + current_acc * time_t
        if clip_vel is not None:
            current_vel = np.clip(current_vel, -clip_vel, clip_vel)

        updated_joint = self._last_pos[-1] + current_vel * time_t
        self._last_vel = np.roll(self._last_vel, -1, axis=0)
        self._last_vel[-1] = current_vel
        self._last_pos = np.roll(self._last_pos, -1, axis=0)
        self._last_pos[-1] = updated_joint
        return updated_joint.tolist()

    def _ema_joint(self, joint, time_t, gamma=1.0, clip_vel=None, clip_acc=None, window_size=10):
        weights = np.exp(-gamma * np.arange(window_size))
        weights /= weights.sum()

        # Calculate the current velocity using finite difference
        current_vel = (np.array(joint) - self._last_pos[-1]) / time_t

        # Update the EMA for velocity
        self._last_vel = np.roll(self._last_vel, -1, axis=0)
        self._last_vel[-1] = np.dot(weights, self._last_vel[-1:-1 - window_size:-1]) + (1 - gamma) * current_vel

        # Calculate the current acceleration using finite difference
        current_acc = (self._last_vel[-1] - self._last_vel[-2]) / time_t
        if clip_acc is not None:
            current_acc = np.clip(current_acc, -clip_acc, clip_acc)

        # Update the EMA for position
        self._last_pos = np.roll(self._last_pos, -1, axis=0)
        self._last_pos[-1] = self._last_pos[-2] + self._last_vel[-2] * time_t + 0.5 * current_acc * time_t ** 2

        if clip_vel is not None:
            # Clip the resulting velocity
            self._last_vel[-1] = np.clip(self._last_vel[-1], -clip_vel, clip_vel)

        updated_joint = self._last_pos[-1]
        return updated_joint.tolist()


def fr5():
    try:
        robot = FR5('192.168.1.10')
    except:
        traceback.print_exc()
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
