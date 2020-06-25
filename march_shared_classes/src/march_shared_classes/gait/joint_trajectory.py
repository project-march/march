import rospy
from scipy.interpolate import BPoly

from .setpoint import Setpoint


class JointTrajectory(object):
    """Base class for joint trajectory of a gait."""

    setpoint_class = Setpoint

    def __init__(self, name, limits, setpoints, duration, *args):
        self.name = name
        self.limits = limits
        self._setpoints = setpoints
        self._duration = duration
        self.interpolated_position = None
        self.interpolated_velocity = None
        self.interpolate_setpoints()

    @classmethod
    def from_dict(cls, subgait_dict, joint_name, limits, duration, *args):
        """Create class of JointTrajectory with filled attributes.

        :param subgait_dict:
            The dictionary extracted from the yaml file
        :param joint_name:
            The name of the joint corresponding to this specific object
        :param limits:
            Defined soft limits of the urdf file
        :param duration:
            The timestamps of the subgait file
        """
        joint_trajectory = subgait_dict['trajectory']
        joint_index = joint_trajectory['joint_names'].index(joint_name)

        setpoints = []
        for point in joint_trajectory['points']:
            time = rospy.Duration(point['time_from_start']['secs'], point['time_from_start']['nsecs']).to_sec()
            setpoints.append(cls.setpoint_class(time, point['positions'][joint_index],
                                                point['velocities'][joint_index]))

        return cls(joint_name, limits, setpoints, duration, *args)

    @property
    def duration(self):
        return self._duration

    def set_duration(self, new_duration, rescale=True):
        for setpoint in reversed(self.setpoints):
            if rescale:
                setpoint.time = setpoint.time * new_duration / self.duration
            else:
                if setpoint.time > new_duration:
                    self.setpoints.remove(setpoint)

        self._duration = new_duration
        self.interpolate_setpoints()

    @property
    def setpoints(self):
        return self._setpoints

    @setpoints.setter
    def setpoints(self, setpoints):
        self._setpoints = setpoints
        self.interpolate_setpoints()

    def get_setpoints_unzipped(self):
        """Get all the listed attributes of the setpoints."""
        time = []
        position = []
        velocity = []

        for setpoint in self.setpoints:
            time.append(setpoint.time)
            position.append(setpoint.position)
            velocity.append(setpoint.velocity)

        return time, position, velocity

    def validate_joint_transition(self, joint):
        """Validate the ending and starting of this joint to a given joint.

        :param joint:
            the joint of the next subgait (not the previous one)

        :returns:
            True if ending and starting point are identical else False
        """
        if not self._validate_boundary_points():
            return False

        from_setpoint = self.setpoints[-1]
        to_setpoint = joint.setpoints[0]

        if from_setpoint.velocity == to_setpoint.velocity and from_setpoint.position == to_setpoint.position:
            return True

        return False

    def _validate_boundary_points(self):
        """Validate the starting and ending of this joint are at t = 0 and t = duration, or that their speed is zero.

        :returns:
            False if the starting/ending point is (not at 0/duration) and (has nonzero speed), True otherwise
        """
        return (self.setpoints[0].time == 0 or self.setpoints[0].velocity == 0) and \
               (self.setpoints[-1].time == round(self.duration, Setpoint.digits) or self.setpoints[-1].velocity == 0)

    def interpolate_setpoints(self):
        if len(self.setpoints) == 1:
            self.interpolated_position = lambda time: self.setpoints[0].position
            self.interpolated_velocity = lambda time: self.setpoints[0].velocity
            return

        time, position, velocity = self.get_setpoints_unzipped()
        yi = []
        for i in range(0, len(time)):
            yi.append([position[i], velocity[i]])

        # We do a cubic spline here, just like the ros joint_trajectory_action_controller,
        # see https://wiki.ros.org/robot_mechanism_controllers/JointTrajectoryActionController
        self.interpolated_position = BPoly.from_derivatives(time, yi)
        self.interpolated_velocity = self.interpolated_position.derivative()

    def get_interpolated_setpoint(self, time):
        if time < 0:
            rospy.logerr('Could not interpolate setpoint at time {0}'.format(time))
            return self.setpoint_class(time, self.setpoints[0].position, 0)
        if time > self.duration:
            rospy.logerr('Could not interpolate setpoint at time {0}'.format(time))
            return self.setpoint_class(time, self.setpoints[-1].position, 0)

        return self.setpoint_class(time, self.interpolated_position(time), self.interpolated_velocity(time))

    def __getitem__(self, index):
        return self.setpoints[index]

    def __len__(self):
        return len(self.setpoints)
