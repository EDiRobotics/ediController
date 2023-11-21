import os
import sys
import time
from ctypes import cdll

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

_robot = frrpc.RPC('192.168.1.10')


class FR5:

    def __init__(self, robot):
        self.robot = robot
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

    def _gripper_ctr(self, EF_Joint):
        try:
            gripper_pos = EF_Joint
            self.robot.SetToolAO(0, float(gripper_pos), 1)
        except Exception as e:
            print(f"[_gripper_ctr] An error occurs:", e)

    def open_gripper(self):
        self._gripper_ctr(800)

    def close_gripper(self):
        self._gripper_ctr(100)

    def set_gripper(self, p):
        if 0 <= p <= 1000:
            self._gripper_ctr(p)
        else:
            print(f"[set_gripper] Gripper control angle {p} is not valid")

    def move_end(self, pose):
        pose = [float(x) for x in pose]
        if pose[2] < 180:
            print(f"[move_end] Robot z value {pose[2]} is dangerous!")
            pose[2] = 200.0
        self.robot.MoveCart(pose, 1, 0, 100.0, 100.0, 100.0, -1.0, -1)

    def move_joint(self, joint):
        if len(joint) != 6:
            print(f"[move_joint] joint is {joint} which has invalid length")
            return
        try:
            J1 = joint
            P1 = _robot.GetForwardKin(J1)[1:]
            eP1 = [0.000, 0.000, 0.000, 0.000]
            dP1 = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]
            _robot.MoveJ(J1, P1, 1, 0, 100.0, 180.0, 100.0, eP1, -1.0, 0, dP1)

        except Exception as e:
            print(f"[move_joint] An error occurs: ", e)


def fr5():
    myRobot = None
    try:
        myRobot = FR5(_robot)
    except:
        exit("Can not connect to robot!")
    return myRobot
    # raise NotImplementedError
