import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .gait_interface import GaitInterface


class HomeGait(GaitInterface):
    def __init__(self, name, position, duration=3.0):
        self._name = 'home_{name}'.format(name=name)
        self._position = position
        self._duration = duration
        self._elapsed_time = 0.0
        self._has_started = False

    def name(self):
        return self._name

    def starting_position(self):
        return self._position

    def final_position(self):
        return self._position

    def start(self):
        self._elapsed_time = 0.0
        self._has_started = False

    def update(self, elapsed_time):
        if not self._has_started:
            self._has_started = True
            return self._get_trajectory_msg(), False

        self._elapsed_time += elapsed_time
        if self._elapsed_time >= self._duration:
            return None, True

    def _get_trajectory_msg(self):
        msg = JointTrajectory()
        msg.joint_names = sorted(list(self._position.keys()))

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(self._duration)
        point.positions = [self._position[name] for name in msg.joint_names]
        point.velocities = [0.0] * len(msg.joint_names)
        point.accelerations = [0.0] * len(msg.joint_names)
        point.effort = [0.0] * len(msg.joint_names)

        msg.points = [point]
        return msg
