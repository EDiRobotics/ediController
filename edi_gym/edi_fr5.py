import os
import sys
import time
from ctypes import cdll
from typing import Dict, List, Tuple
import numpy as np

try:
    from .errors import robot_errors
except:
    sys.path.append(".")
    from errors import robot_errors

# Get the directory containing this script
current_dir = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(current_dir, "frrpc.so")

if not os.path.exists(path):
    print("Please put 'frrpc.so' into the file directory.")
    raise FileNotFoundError

# Add the directory containing frrpc.so to sys.path
sys.path.append(current_dir)
# Load the frrpc library
frrpc = cdll.LoadLibrary(path)
# Now you can import frrpc and use it
import frrpc


class FR5:
    z_thres = 135
    method = "servoj"

    _lastHandlerJoint = np.zeros(6)
    _lastJointsVel = np.zeros(6)
    _filter_width = 6
    _trajVel = [np.zeros(6)] * _filter_width
    _first = True
    _cmdT = 0.12

    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.robot = frrpc.RPC(self.robot_ip)
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
        print(f'\033[37m[__init__]: Robot initialized. \033[0m')

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

    def move_joint(self, joint):
        if len(joint) != 6:
            print(f"[move_joint] joint is {joint} which has invalid length")
            return 3
        loc = self.robot.GetForwardKin(joint)[1:]
        if loc[2] < self.z_thres:
            print(f"[move_joint] joint is {joint}, which is too low at {loc}, force return.")
            return 14
        if self.method.lower() == "movej" or self._first is True:
            try:
                j1 = joint
                p1 = self.robot.GetForwardKin(j1)[1:]
                e_p1 = [0.000, 0.000, 0.000, 0.000]
                d_p1 = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]
                print(f"Debug: [move_joint] move joint to {joint}, P1 {p1}.")
                self._first = False
                self._lastHandlerJoint = joint
                return self.robot.MoveJ(j1, p1, 1, 0, 100.0, 180.0, 100.0, e_p1, -1.0, 0, d_p1)

            except Exception as e:
                print(f"[move_joint] An error occurs: ", e)
        else:
            joint = self._emaFilterVel(joint, self._lastHandlerJoint)
            try:
                return self.robot.ServoJ(joint, 0.0, 0.0, self._cmdT, 0.0, 0.0)
            except Exception as e:
                print(f"[move_joint] An error occurs: ", e)

    def detect_errors(self) -> Tuple[int, List[Dict[int, str]]]:
        rets = self.robot.GetRobotErrorCode()
        errors = [{ret: robot_errors[str(ret)]} for ret in rets]
        e = 0 if all(ret == 0 for ret in rets) else 1
        return e, errors

    def lookup_error(self, ret):
        return {ret: robot_errors[str(ret)]}

    def clear_errors(self):
        return self.robot.ResetAllError()

    def _emaFilterVel(self, handler_joints, last_handler_joints, gamma=0.12, clip_th=2.0):
        handler_joints = np.array(handler_joints)
        last_handler_joints = np.array(last_handler_joints)
        joints_vel = handler_joints - last_handler_joints
        joints_vel = self._trajVel[-1] * (1.0 - gamma) + joints_vel * gamma
        if np.max(joints_vel) > clip_th:
            print(f'\033[0;41;41m[move_joint] EmaFilter: joints_vel {np.max(joints_vel)} over clip_th {clip_th}\033[0m')
        joints_vel = np.clip(joints_vel, -clip_th, clip_th)
        self._trajVel[-1] = joints_vel
        handler_joints = last_handler_joints + joints_vel
        max_acc = max(abs(joints_vel - self._lastJointsVel) / 0.008)
        if max_acc > 100.0:
            print(f'\033[0;41;41m[move_joint] EmaFilter: max acc {max_acc}\033[0m')
        self._lastJointsVel = joints_vel
        self._lastHandlerJoint = handler_joints
        handler_joints = handler_joints.tolist()
        return handler_joints


def fr5():
    myRobot = None
    try:
        myRobot = FR5('192.168.1.10')
    except:
        exit("Can not connect to robot!")
    return myRobot
    # raise NotImplementedError


if __name__ == "__main__":
    robot = fr5()
