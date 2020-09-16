import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .gait_interface import GaitInterface


class HomeGait(GaitInterface):
    def __init__(self, name, position, gait_type, duration=3.0):
        """Initializes an executable home gait with given positions.

        :param str name: Name of the idle position this gait homes to. Will be prefixed with `home_`
        :param dict position: Mapping of joint names to positions
        :param str gait_type: Gait type to use for home gait
        :param float duration: Duration of the gait in seconds. Defaults to 3 seconds.
        """
        self._name = 'home_{name}'.format(name=name)
        self._position = position
        self._gait_type = gait_type
        self._duration = duration
        self._time_since_start = 0.0

    @property
    def name(self):
        return self._name

    @property
    def subgait_name(self):
        return self._name

    @property
    def duration(self):
        return self._duration

    @property
    def gait_type(self):
        return self._gait_type

    @property
    def starting_position(self):
        return None

    @property
    def final_position(self):
        return self._position

    def start(self):
        self._time_since_start = 0.0
        return self._get_trajectory_msg()

    def update(self, elapsed_time):
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
