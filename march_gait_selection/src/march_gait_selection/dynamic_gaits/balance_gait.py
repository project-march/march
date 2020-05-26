
from copy import deepcopy
import sys

import moveit_commander
import rospy
from visualization_msgs.msg import Marker

from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.gait.subgait import Subgait
from march_shared_resources import msg


class BalanceGait(object):
    """Base class to create a gait using the moveit motion planning."""

    def __init__(self, gait_name='gait_balanced_walk'):
        self.gait_name = gait_name
        self.default_walk = None
        self._is_balance_used = rospy.get_param('/balance', False)

        self._move_group = None
        if self._is_balance_used:
            moveit_commander.roscpp_initialize(sys.argv)
            moveit_commander.RobotCommander()
            moveit_commander.PlanningSceneInterface()

            try:
                self._move_group = {'left_leg': moveit_commander.MoveGroupCommander('left_leg'),
                                    'right_leg': moveit_commander.MoveGroupCommander('right_leg')}
            except RuntimeError:
                rospy.logerr('Could not connect to move groups, aborting initialisation of the moveit subgait class')
                return

        self._end_effectors = {'left_leg': 'foot_left', 'right_leg': 'foot_right'}
        self._capture_point_pose = {'left_leg': None, 'right_leg': None}

        self._latest_capture_point_msg_time = {'left_leg': None, 'right_leg': None}
        self._old_capture_point_msg_time = {'left_leg': None, 'right_leg': None}

        rospy.Subscriber('/march/cp_marker_foot_left', Marker, queue_size=1, callback=self.capture_point_cb,
                         callback_args='left_leg')
        rospy.Subscriber('/march/cp_marker_foot_right', Marker, queue_size=1, callback=self.capture_point_cb,
                         callback_args='right_leg')

    def capture_point_cb(self, msg, leg_name):
        """Set latest message to variable.

        :param msg: The message from the capture point topic
        :param leg_name: The name of corresponding move group
        """
        self._latest_capture_point_msg_time[leg_name] = msg.header.stamp
        self._capture_point_pose[leg_name] = msg.pose

    def random_subgait(self):
        """Create random trajectory using the moveit motion planner.

        :return:
            A populated subgait message
        """
        self._move_group['left_leg'].set_random_target()
        trajectory_msg = self._move_group['left_leg'].plan()
        return self.to_subgait_msg('Random moveit subgait', trajectory_msg.joint_trajectory)

    def calculate_trajectory(self, leg_name):
        """Calculate the trajectory using moveit and return as a subgait msg format.

        :param leg_name: The name of the used move group

        :return:
            A joint trajectory which can be used in a subgait or subgait msg
        """
        if self._capture_point_pose[leg_name] is None:
            rospy.logwarn('No messages received from the capture point topic of {lg}'.format(lg=leg_name))
            return None

        if self._old_capture_point_msg_time[leg_name]:
            if self._old_capture_point_msg_time[leg_name] == self._latest_capture_point_msg_time[leg_name]:
                rospy.logwarn('No new capture point messages received from cp topic of {lg}'.format(lg=leg_name))
                return None

        self._old_capture_point_msg_time[leg_name] = self._latest_capture_point_msg_time[leg_name]

        _pose = self._capture_point_pose[leg_name]
        _end_effector = self._end_effectors[leg_name]
        self._move_group[leg_name].set_joint_value_target(_pose, _end_effector, True)

        trajectory_plan = self._move_group[leg_name].plan()
        return trajectory_plan.joint_trajectory

    @staticmethod
    def to_subgait_msg(name, trajectory_msg, gait_type='walk_like', version='moveit',
                       description='Subgait created using the moveit motion planning.'):
        """Create a subgait message using the standard format in the march shared resources."""
        subgait_msg = msg.Subgait()
        subgait_msg.name = name
        subgait_msg.description = description
        subgait_msg.trajectory = trajectory_msg
        subgait_msg.version = version
        subgait_msg.gait_type = gait_type
        subgait_msg.duration = subgait_msg.trajectory.points[-1].time_from_start

        return subgait_msg

    @staticmethod
    def to_subgait(joints, duration, gait_name='balance_gait', gait_type='walk_like', version='moveit',
                   subgait_name='balance_subgait', description='Subgait created using the moveit motion planning.'):
        """Create a subgait using the standard format in the march shared classes."""
        return Subgait(joints, duration, gait_type, gait_name, subgait_name, version, description)

    @staticmethod
    def create_subgait_of_trajectory(normal_subgait, joint_trajectory):
        balance_duration = joint_trajectory.points[-1].time_from_start.to_sec()

        balance_joints = []
        for joint_index, joint_name in enumerate(joint_trajectory.joint_names):
            normal_joint = normal_subgait.get_joint(joint_name)

            setpoints = []
            for joint_trajectory_point in joint_trajectory.points:
                time = joint_trajectory_point.time_from_start.to_sec()
                position = joint_trajectory_point.positions[joint_index]
                velocity = joint_trajectory_point.velocities[joint_index]
                setpoints.append(Setpoint(time, position, velocity))

            balance_joints.append(JointTrajectory(joint_name, normal_joint.limits, setpoints, balance_duration))

        balance_subgait = BalanceGait.to_subgait(balance_joints, balance_duration)

        return balance_subgait

    def construct_subgait(self, leg_name, subgait_name):
        capture_point_trajectory = self.calculate_trajectory(leg_name)

        if not capture_point_trajectory:
            return None

        default_subgait = deepcopy(self.default_walk[subgait_name])
        self.create_subgait_of_trajectory(default_subgait, capture_point_trajectory)

        return self.default_walk[subgait_name]

    def __getitem__(self, name):
        """Return the trajectory of a move group based on capture point in subgait msg format.

        :param name: the name of the subgait (in this case only left_swing and right_swing should be used)
        """
        if name == 'left_swing':
            return self.construct_subgait('left_leg', 'left_swing')
        elif name == 'right_swing':
            return self.construct_subgait('right_leg', 'right_swing')
        else:
            return self.default_walk[name]
