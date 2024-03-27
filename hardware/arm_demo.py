import json
import pdb
import sys
import json
import numpy as np
from socket import socket, AF_INET, SOCK_DGRAM
import codecs
from pymodbus.utilities import computeCRC
import time
import codecs
import rospy

sys.path.append(r'.')

from edi_gym.edi_env import EdiEnv

J5_min = 100.0
J5_max = 220.0
EF_min = -60.0
EF_max = 60.0


# from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
# from backend.srv import StringService, StringServiceRequest, StringServiceResponse
# service = '/env/step/demo_action_srv'
# rospy.wait_for_service(service)
# action_service = rospy.ServiceProxy(service, StringService)
# rospy.loginfo(f"Server {service} is ready...")
#
#
# def execute_action(action):
#     if isinstance(action, np.ndarray):
#         action = action.tolist()
#     action_json = json.dumps(action)
#     request = StringServiceRequest(action_json)
#     rospy.logdebug(f"Request action: {action_json}")
#     response = action_service(request)
#     if not response.success:
#         rospy.logerr(f"Request action return errors: {response.message}")


def crc(data):
    return computeCRC(data).to_bytes(2, 'big')


def clamp_map(value, f_min, f_max, t_min, t_max):  # , bias = 0):
    if value > f_max:
        value = f_max
    elif value < f_min:
        value = f_min

    # value = value + bias
    # f_min = f_min + bias
    # f_max = f_max + bias

    # print(value, f_min, f_max, t_min, t_max, (value - f_min) / (f_max - f_min) * (t_max - t_min) + t_min)
    value = (value - f_min) / (f_max - f_min) * (t_max - t_min) + t_min

    # if value > t_max:
    #     print(value, "t_max", t_max)
    #     value = t_max
    # elif value < t_min:
    #     print(value, "t_min", t_min)
    #     value = t_min

    return value


class UDP(object):

    def __init__(self, ipAddress, port):
        self.UDP_CLIENT_SOCKET = socket(AF_INET, SOCK_DGRAM)
        self.IP_ADDR = ipAddress  # Arm IP
        self.PORT = port
        self.Address = (ipAddress, port)

        self.LocalAddress = ("", 2001)
        self.UDP_CLIENT_SOCKET.bind(self.LocalAddress)
        print("UDP server " + ipAddress + ":" + str(port) + " connected")

    def Send(self, stream):
        self.UDP_CLIENT_SOCKET.sendto(stream, self.Address)
        # print("SEND DATA:" + str(codecs.encode(stream, 'hex')))

    def Request(self, stream):
        self.UDP_CLIENT_SOCKET.sendto(stream, self.Address)
        receive, client = self.UDP_CLIENT_SOCKET.recvfrom(48)
        # print("SEND DATA:" + str(codecs.encode(receive, 'hex')))
        return codecs.encode(receive, 'hex')

    def Close(self):
        self.UDP_CLIENT_SOCKET.close()


class ArmINNFO(object):
    CMD_ID = {
        'ENABLE': b'\x2A',
        'GET_JOINTS_DEGREE': b'\x06'
    }

    def __init__(self):
        self.Joints = [0, 0, 0, 0, 0, 0]

        self.udp = UDP("192.168.1.30", 2000)
        self.Q24 = pow(2, 24)  # IQMath IQ24
        if self.udp.Request(b'\xEE\x00\x44\x00\x00\xED') == b'ee00440001010000ed':
            rospy.logdebug("Arm INNFO connected.")

        self.udp.Send(b'\xEE\x00\x02\x00\x00\xED')
        rospy.logdebug("Arm INNFO initialized.")

        self.SetAllServoEnable(True)
        rospy.logdebug("Arm INNFO all servos enabled.")

    # command for write
    def CommandBuildCRC(self, servo_id, cmd_id, data):
        head = b'\xEE'
        servo = servo_id.to_bytes(1, "little")
        dataLenth = len(data).to_bytes(2, "big")
        dataCRC = crc(data)
        end = b'\xED'

        return head + servo + cmd_id + dataLenth + data + dataCRC + end

    # command for read
    def CommandBuild(self, servo_id, cmd_id):
        head = b'\xEE'
        servo = servo_id.to_bytes(1, "little")
        dataLenth = b'\x00\x00'
        end = b'\xED'

        return head + servo + cmd_id + dataLenth + end

    def SetServoEnable(self, servo_id, enable):
        if enable:
            data = b'\x01'
        else:
            data = b'\x00'

        cmd = self.CommandBuildCRC(servo_id, self.CMD_ID["ENABLE"], data)
        self.udp.Send(cmd)

    def SetAllServoEnable(self, enable):
        if enable:
            data = b'\x01'
        else:
            data = b'\x00'

        for i in range(1, 7):
            cmd = self.CommandBuildCRC(i, self.CMD_ID["ENABLE"], data)
            self.udp.Send(cmd)
            rospy.logdebug("Servo " + str(i) + " enabled")

    def GetServoDegree(self):
        # import numpy as np
        # last_Joints = np.array(self.Joints)
        for i in range(1, 7):
            cmd = self.CommandBuild(i, self.CMD_ID["GET_JOINTS_DEGREE"])
            recv = self.udp.Request(cmd)
            rawValue = int(recv[10:18], 16) / self.Q24

            # recv[3]
            if int(recv[18:22], 16) != int(computeCRC(int(recv[10:18], 16).to_bytes(4, 'big'))):
                print(i, int(recv[18:22], 16),
                      int(computeCRC(int(recv[10:18], 16).to_bytes(4, 'big'))))  # .to_bytes(2, 'little'))
            # print('0x%x'%recv[0],'0x%x'%recv[1],'0x%x'%recv[2],'0x%x'%recv[3],'0x%x'%recv[4])
            # print(recv)#[0],recv[1],recv[2],recv[3],recv[4])
            # print(i, recv[3] - 49)
            # if i-1 != recv[3] - 49:
            #     print(i, recv, time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime()))
            #     exit()

            if rawValue <= 256 and rawValue >= 128:
                rawValue -= 256
            # self.Joints[i-1] = round(rawValue * 10, 6)
            self.Joints[recv[3] - 49] = round(rawValue * 10, 6)
        return self.Joints


lastJoint6 = None


def JointsMap(handlerJoints):
    global lastJoint6
    if lastJoint6 == None:
        lastJoint6 = clamp_map(handlerJoints[5], J5_min, J5_max, J5_min, J5_max)

    if abs(handlerJoints[5] - lastJoint6) < 3.1 and J5_min < handlerJoints[5] and handlerJoints[
        5] < J5_max:
        # Handler6 = handlerJoints[5]
        lastJoint6 = handlerJoints[5]

    # [180, -180]    [-175, 175]
    # [90, -110]    [-265, 85]  [-190, 10]
    # [-20, 280]    [-160, 160] []
    # [150, -152]    [-265, 85] []
    # [-90, 270]    [-175, 175]
    # [-20, 160]    [-175, 175]
    map_array = np.array([round(clamp_map(handlerJoints[0], -170, 170, 170, -170), 6),  # + 15, 6),
                          round(clamp_map(handlerJoints[1], -100, 90, 10, -180), 6),  # - 75, 6),
                          round(handlerJoints[2] - 116, 6),  # - 116, 6),
                          round(clamp_map(handlerJoints[3], -180, 120, 80, -260), 6),  # - 92, 6),
                          round(clamp_map(handlerJoints[4], -90, 270, 170, -170), 6),  # + 170, 6), # -90
                          round(clamp_map(lastJoint6, J5_min, J5_max, -165, 165), 6),  # ])
                          round(clamp_map(handlerJoints[5], EF_min, EF_max, 0, 1000),
                                6)])  # -20, 150   # [0~4095]
    # handlerJoints[5]])
    # round(currentJoints[6], 6)])
    # [-175, 175]
    # armFR.J6]
    # print(handlerJoints, map_array)

    # map(handlerJoints[5], EF_min, EF_max, 0, 1000)
    # return Joints, EF
    # print('map_array: ', map_array)
    return map_array


armINNFO = ArmINNFO()

time.sleep(1.5)
handlerJoints = armINNFO.GetServoDegree()
time.sleep(0.5)
handlerJoints = armINNFO.GetServoDegree()
time.sleep(0.1)
handlerJoints = armINNFO.GetServoDegree()

print("INNFO initialize finished.", handlerJoints)
handlerJoints = armINNFO.GetServoDegree()
handlerJoints = JointsMap(handlerJoints)

begin_time = time.time()
print("start the joints loop at: ", begin_time)

env = EdiEnv(demo=True)
env.reset()
rospy.set_param("/env/ctrl/switch", "demo")

while not rospy.is_shutdown():
    handlerJoints = armINNFO.GetServoDegree()
    handlerJoints = JointsMap(handlerJoints)
    action = handlerJoints
    step_start = rospy.Time.now()
    obs, _, _, info = env.step(action)
    rospy.logdebug(f"[Demo] Step time {(rospy.Time.now() - step_start).to_sec()}")
