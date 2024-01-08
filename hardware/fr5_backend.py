#!/usr/bin/env python
import abc
import json
import pdb
import queue
import sys
import threading
import time
import traceback
from abc import abstractmethod
import numpy as np

sys.path.append(".")
import rospy
from std_msgs.msg import String, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse
from backend.srv import StringService, StringServiceRequest, StringServiceResponse
from hardware.arm_fr5 import fr5

try:
    robot_controller = fr5()
except Exception as e:
    rospy.logerr(f"Error occurs when launching Real Environment Backend: {e}")
    exit(0)

rospy.init_node('backend', anonymous=True)
rospy.loginfo(f"Launching Real Environment Backend...")

control_freq = 100
control_t = 1 / control_freq
control_t_nsecs = int(control_t * 1000000000)
idx = 0
obs_time = rospy.Time.now()


def step_with_action_servo(action, cmd_time):
    """
    :param action: 7 length list. The first 6 item contain the 6 joint angle,
    :param cmd_time:
    the last item contains the gripper proportion.
    :return: a dict containing some information
    """
    joint = action[:6]
    gripper = action[-1]
    publisher_gripper.publish(gripper)
    # robot_controller.move_joint(joint)
    robot_controller.move_joint_servo(joint, cmd_time)
    # robot_controller.set_gripper(gripper)


topic_name = "/arm_status/gripper_pos"
publisher_gripper = rospy.Publisher(topic_name, Float32, queue_size=5)
topic_name = "/env/step/idx"
publisher_idx = rospy.Publisher(topic_name, Int32, queue_size=100)
topic_name = "/env/step/action"
publisher_action = rospy.Publisher(topic_name, String, queue_size=100)
topic_name = "/env/step/info"
publisher_info = rospy.Publisher(topic_name, String, queue_size=100)


class Trajectory(abc.ABC):

    def __init__(self, traj_start):
        self.traj_start = traj_start

    @abstractmethod
    def get_position(self, t):
        """
        :return: ndarray (7,)
        """
        return None

    @property
    def start(self):
        """
        :return: rospy.Time
        """
        return self.traj_start

    @property
    def end(self):
        """
        :return: rospy.Time
        """
        return None


class DiscreteTrajectory(Trajectory):
    def __init__(self, traj_start, plans, pace_duration):
        """
        :param traj_start: rospy.Time
        :param plans: List of positions, each position is ndarray (7,)
        :param pace_duration:  rospy.Duration
        """
        super().__init__(traj_start)  # Call the base class's __init__ method
        self.plans = plans
        self.pace_duration = pace_duration

    def get_position(self, t):
        if t < self.start or t > self.end:
            return None

        # Calculate the step index
        time_elapsed = t - self.start
        step_index = int(time_elapsed // self.pace_duration)

        # If it's the last step, return the last position
        if step_index >= len(self.plans) - 1:
            return self.plans[-1]

        # Calculate the fraction of the current step
        step_fraction = (time_elapsed % self.pace_duration) / self.pace_duration

        # Linear interpolation between the current step and the next step
        current_step_position = self.plans[step_index]
        next_step_position = self.plans[step_index + 1]
        interpolated_position = current_step_position + step_fraction * (next_step_position - current_step_position)

        return interpolated_position

    @property
    def end(self):
        return self.start + len(self.plans) * self.pace_duration

    def get_last(self):
        return self.plans[-1]


class TrajectoryServer:
    """
    Suppose the trajectory optimizer is de
    """

    def __init__(self, optimize_interval=0.02, traj_length=0.1, config=None):
        if config is None:
            config = {}
        self.config = config
        self.optimize_interval = rospy.Duration(optimize_interval)
        self.action_queue = queue.Queue()
        self.current_window = None
        self.traj_length = rospy.Duration(traj_length)
        self.optimized_trajectory = None
        self.first_trajectory = None
        self.last_position = None
        self.raw_trajectories = []
        self.thread = threading.Thread(target=self.reoptimize_traj, args=())
        self.thread.start()

    def reoptimize_traj(self):
        while not rospy.is_shutdown():
            self._optimize_traj()
            rospy.sleep(self.optimize_interval)

    def _optimize_traj(self):
        pace_duration = rospy.Duration(nsecs=control_t_nsecs)
        while not self.action_queue.empty():
            action_plans, timestamp = self.action_queue.get()
            start_timestamp = timestamp + rospy.Duration(nsecs=control_t_nsecs)
            self.raw_trajectories.append(DiscreteTrajectory(start_timestamp, action_plans, pace_duration))
        now = rospy.Time.now()
        self.raw_trajectories = [traj for traj in self.raw_trajectories if traj.end > now]
        end_time = now + self.traj_length
        current_time = now
        pace_duration = rospy.Duration(nsecs=control_t_nsecs)
        ema_positions = []

        while current_time <= end_time:
            valid_positions = []
            weights = []
            total_weight = 0

            # Gather positions and calculate weights
            for traj in self.raw_trajectories:
                position = traj.get_position(current_time)
                if position is not None:
                    valid_positions.append(position)
                    weight = np.exp(-(now - traj.start).to_sec())
                    weights.append(weight)
                    total_weight += weight

            # Normalize weights and calculate weighted sum
            if total_weight > 0 and valid_positions:
                normalized_weights = [w / total_weight for w in weights]
                weighted_sum = sum(np.array(pos) * w for pos, w in zip(valid_positions, normalized_weights))
                ema_positions.append(weighted_sum)
            else:
                break

            current_time += pace_duration
        if ema_positions:
            self.first_trajectory = DiscreteTrajectory(now, ema_positions, pace_duration)
            self.optimized_trajectory = self._spline_optimize_traj(self.first_trajectory)

    def _spline_optimize_traj(self, first_trajectory):
        optimize_config = self.config
        return first_trajectory

    def add_action(self, action):
        now = rospy.Time.now()
        action_: np.ndarray = np.array(action)
        if len(action_.shape) == 1:
            action_ = action_[np.newaxis, :]
        self.last_position = action_[-1]
        self.action_queue.put((action_, now))

    def get_action(self, t):
        if self.optimized_trajectory is None:
            return None
        action = self.optimized_trajectory.get_position(t)
        if action is not None:
            return action.tolist()
        rospy.logwarn("self.optimized_trajectory.get_position failed, use last action. ")
        if self.last_position is not None:
            return self.last_position.tolist()
        return None


class SimpleTrajectoryServer(TrajectoryServer):

    def _optimize_traj(self):
        pace_duration = rospy.Duration(nsecs=control_t_nsecs)
        while not self.action_queue.empty():
            action_plans, timestamp = self.action_queue.get()
            start_timestamp = timestamp + rospy.Duration(nsecs=control_t_nsecs)
            if len(action_plans) == 1:
                n = 20
                action_plans = np.tile(action_plans, (n, 1))
            self.raw_trajectories.append(DiscreteTrajectory(start_timestamp, action_plans, pace_duration))
        now = rospy.Time.now()
        self.raw_trajectories = [traj for traj in self.raw_trajectories if traj.end > now]
        end_time = now + self.traj_length
        current_time = now
        pace_duration = rospy.Duration(nsecs=control_t_nsecs)
        ema_positions = []

        while current_time <= end_time:
            valid_positions = []
            weights = []
            total_weight = 0

            # Gather positions and calculate weights
            for traj in self.raw_trajectories:
                position = traj.get_position(current_time)
                if position is not None:
                    valid_positions.append(position)
                    weight = np.exp(-(now - traj.start).to_sec())
                    weights.append(weight)
                    total_weight += weight

            # Normalize weights and calculate weighted sum
            if total_weight > 0 and valid_positions:
                normalized_weights = [w / total_weight for w in weights]
                weighted_sum = sum(np.array(pos) * w for pos, w in zip(valid_positions, normalized_weights))
                ema_positions.append(weighted_sum)
            else:
                # TODO
                if self.last_position is not None:
                    ema_positions.append(self.last_position)
                else:
                    break

            current_time += pace_duration
        if ema_positions:
            self.first_trajectory = DiscreteTrajectory(now, ema_positions, pace_duration)
            self.optimized_trajectory = self._spline_optimize_traj(self.first_trajectory)


traj_server = SimpleTrajectoryServer()

joint_angles = [[] for _ in range(6)]
times = []
plot_lock = threading.Lock()


class ArmControlThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.control_time = None

    def run(self):
        call_count = 0
        loop_count = 0
        start_time = time.time()
        robot_controller.reset_first()
        self.control_time = rospy.Time.now()
        while not rospy.is_shutdown():
            loop_count += 1
            elapsed_time = time.time() - start_time
            if elapsed_time >= 20:  # Check if 60 seconds have passed
                frequency = call_count / elapsed_time
                loop_frequency = loop_count / elapsed_time
                rospy.loginfo(f"Actual frequency: {frequency} Hz, Loop frequency: {loop_frequency}")
                call_count = 0
                loop_count = 0
                start_time = time.time()
            next_control_time = self.control_time + rospy.Duration(nsecs=control_t_nsecs)
            action = traj_server.get_action(next_control_time)

            if action is None:
                rospy.logerr("Get action is None!")
                self.control_time = rospy.Time.now()
                continue
            if rospy.Time.now() < next_control_time:
                sleep_duration = next_control_time - rospy.Time.now()
                rospy.sleep(sleep_duration)
                self.control_time = next_control_time
            else:
                rospy.logwarn("Missing desired control frequency...")
                self.control_time = rospy.Time.now()
            try:
                call_count += 1
                step_with_action_servo(action, control_t)
            except Exception as e:
                rospy.logwarn(f"Error happened: {e}")
                self.control_time = rospy.Time.now()


control_thread = ArmControlThread()
control_thread.start()


def handle_service(request: StringServiceRequest):
    global idx, last_switch_state, obs_time
    info = {"error": 0}
    try:
        action_str = request.message
        action = json.loads(action_str)
        action = [float(a) for a in action]
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Error json loads: {e}")
        error_msg = json.dumps({"error": "JSON decode error"})
        return StringServiceResponse(success=False, message=error_msg)
    if not robot_controller.check_valid(action):
        rospy.logwarn(f"Action {action} is not valid...")
        info["error"] = 4
        error_msg = json.dumps(info)
        return StringServiceResponse(success=False, message=error_msg)
    # FIXME: Multi-thread problem
    # action = robot_controller.revise_action(action)
    idx += 1
    publisher_idx.publish(idx)
    action_time = rospy.Time.now()
    action_record = {"idx": idx, "action": action,
                     "timestamp": action_time.to_time(),
                     "obs_timestamp": obs_time.to_time(),
                     "mode": last_switch_state}
    publisher_action.publish(json.dumps(action_record))
    traj_server.add_action(action)
    obs_time = rospy.Time.now()
    idx += 1
    publisher_idx.publish(idx)
    info_str = json.dumps(info)
    return StringServiceResponse(
        success=True,
        message=info_str
    )


def handle_service_demo(request):
    if rospy.get_param("/env/ctrl/demo", False):
        return handle_service(request)
    else:
        return StringServiceResponse(success=False, message="Not Allowed")


def handle_service_policy(request):
    if rospy.get_param("/env/ctrl/policy", False):
        return handle_service(request)
    else:
        return StringServiceResponse(success=False, message="Not Allowed")


def handle_service_replay(request):
    if rospy.get_param("/env/ctrl/replay", False):
        return handle_service(request)
    else:
        return StringServiceResponse(success=False, message="Not Allowed")


last_switch_state = rospy.get_param("/env/ctrl/switch", "policy")
rospy.set_param("/env/ctrl/switch", last_switch_state)
rospy.set_param("/env/ctrl/policy", False)
rospy.set_param("/env/ctrl/demo", False)
rospy.set_param("/env/ctrl/replay", False)
rospy.set_param(f"/env/ctrl/{last_switch_state}", True)


def switch(event):
    global last_switch_state
    current_switch_state = rospy.get_param("/env/ctrl/switch")
    if current_switch_state not in ["policy", "demo", "replay"]:
        current_switch_state = last_switch_state
        rospy.set_param("/env/ctrl/switch", current_switch_state)

    if current_switch_state != last_switch_state:
        last_switch_state = current_switch_state

        robot_controller.reset_first()
        new_policy_state = (current_switch_state == "policy")
        new_demo_state = (current_switch_state == "demo")
        new_replay_state = (current_switch_state == "replay")

        rospy.set_param("/env/ctrl/policy", new_policy_state)
        rospy.set_param("/env/ctrl/demo", new_demo_state)
        rospy.set_param("/env/ctrl/replay", new_replay_state)
        robot_controller.reset_first()

        if new_policy_state:
            rospy.loginfo("Policy is now enabled.")
        else:
            rospy.loginfo("Policy is now disabled.")

        if new_demo_state:
            rospy.loginfo("Demo is now enabled.")
        else:
            rospy.loginfo("Demo is now disabled.")

        if new_replay_state:
            rospy.loginfo("Replay is now enabled.")
        else:
            rospy.loginfo("Replay is now disabled.")

    assert (int(rospy.get_param("/env/ctrl/policy")) +
            int(rospy.get_param("/env/ctrl/demo")) +
            int(rospy.get_param("/env/ctrl/replay")) <= 1
            )


def rst_service(request):
    try:
        robot_controller.clear_errors()
        robot_controller.reset_first()
        return TriggerResponse(
            success=True,
            message=""
        )
    except Exception as e:
        return TriggerResponse(
            success=False,
            message=f"{str(e)}"
        )


service_rst = rospy.Service('/env/reset_srv', Trigger, rst_service)
service_replay = rospy.Service('/env/step/replay_action_srv', StringService, handle_service_replay)
service_demo = rospy.Service('/env/step/demo_action_srv', StringService, handle_service_demo)
service_policy = rospy.Service('/env/step/policy_action_srv', StringService, handle_service_policy)
timer = rospy.Timer(rospy.Duration(nsecs=100), switch)
rospy.loginfo(f"All servers registered...")


def spin():
    rospy.spin()


spin_thread = threading.Thread(target=spin)
spin_thread.start()

# plt.ion()
# fig, axs = plt.subplots(6, 1, figsize=(10, 15))
#
#
# def plot_joint_angles():
#     current_time = time.time()
#     if times:
#         with plot_lock:
#             recent_times = [t - current_time for t in times if current_time - t <= 2]
#             all_recent_angles = []
#             if recent_times:
#                 for i in range(6):
#                     recent_angles = joint_angles[i][-len(recent_times):]
#                     all_recent_angles.append(recent_angles)
#         if recent_times:
#             for i in range(6):
#                 axs[i].clear()
#                 recent_angles = all_recent_angles[i]
#                 axs[i].plot(recent_times, recent_angles)
#                 axs[i].set_xlim(recent_times[0], recent_times[-1])
#                 axs[i].set_ylim(min(recent_angles), max(recent_angles))
#                 axs[i].set_ylabel(f'Joint {i + 1} Angles')
#             plt.pause(0.01)  # 短暂暂停以更新图表
#     else:
#         print("No data to plot.")
#
#
# print("Start plotting")
#
#
# def loop_plot_joint_angles():
#     while not rospy.is_shutdown():
#         plot_joint_angles()
#
#
# # 在单独的线程中运行绘图函数
# plot_thread = threading.Thread(target=loop_plot_joint_angles)
# plot_thread.start()
# plt.show()
spin_thread.join()
