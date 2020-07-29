import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .gait_interface import GaitInterface


class HomeGait(GaitInterface):
    def __init__(self, name, position, duration=3.0):
        """Initializes an executable home gait with given positions.

        :param str name: Name of the idle position this gait homes to. Will be prefixed with `home_`
        :param dict position: Mapping of joint names to positions
        :param float duration: Duration of the gait in seconds. Defaults to 3 seconds.
        """
        self._name = 'home_{name}'.format(name=name)
        self._position = position
        self._duration = duration
        self._time_since_start = 0.0
        self._has_started = False

    @property
    def name(self):
        return self._name

    @property
    def starting_position(self):
        return None

    @property
    def final_position(self):
        return self._position

    def start(self):
        self._time_since_start = 0.0
        self._has_started = False

    def update(self, elapsed_time):
        if not self._has_started:
            self._has_started = True
            return self._get_trajectory_msg(), False

        self._time_since_start += elapsed_time
        if self._time_since_start >= self._duration:
            return None, True
        else:
            return None, False

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
