from copy import deepcopy
import sys

import moveit_commander
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.gait.subgait import Subgait


class BalanceGait(object):
    """Base class to create a gait using the moveit motion planning."""

    def __init__(self, gait_name='gait_balanced_walk', move_groups=None):
        self.gait_name = gait_name

        self._default_walk = None
        self._move_group = move_groups

        self._end_effectors = {'left_leg': 'foot_left', 'right_leg': 'foot_right'}
        self._capture_point_pose = {'left_leg': None, 'right_leg': None}

        self._latest_capture_point_msg_time = {'left_leg': None, 'right_leg': None}
        self._old_capture_point_msg_time = {'left_leg': None, 'right_leg': None}

    @classmethod
    def create_balance_subgait(cls):
        """This class method should check if the balance variable is set and if so configure the motion planner."""
        is_balance_used = rospy.get_param('/balance', False)

        if not is_balance_used:
            return cls()

        moveit_commander.roscpp_initialize(sys.argv)
        moveit_commander.RobotCommander()

        moveit_commander.PlanningSceneInterface()

        try:
            move_groups = {'all_legs': moveit_commander.MoveGroupCommander('all_legs'),
                           'left_leg': moveit_commander.MoveGroupCommander('left_leg'),
                           'right_leg': moveit_commander.MoveGroupCommander('right_leg')}
        except RuntimeError:
            rospy.logerr('Could not connect to move groups, aborting initialisation of the moveit subgait class')
            return cls()

        rospy.Subscriber('/march/cp_marker_foot_left', Marker, queue_size=1, callback=cls.capture_point_cb,
                         callback_args='left_leg')
        rospy.Subscriber('/march/cp_marker_foot_right', Marker, queue_size=1, callback=cls.capture_point_cb,
                         callback_args='right_leg')

        return cls(move_groups=move_groups)

    @property
    def default_walk(self):
        """Return the default walk subgait."""
        return self._default_walk

    @default_walk.setter
    def default_walk(self, new_default_walk):
        """Set a new default walk subgait to the balance subgait.

        :param new_default_walk:
            A new subgait which is the default balance walking pattern
        """
        self._default_walk = new_default_walk

    def capture_point_cb(self, msg, leg_name):
        """Set latest message to variable.

        :param msg: The message from the capture point topic
        :param leg_name: The name of corresponding move group
        """
        self._latest_capture_point_msg_time[leg_name] = msg.header.stamp
        self._capture_point_pose[leg_name] = msg.pose

    def calculate_capture_point_trajectory(self, leg_name):
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

        pose = self._capture_point_pose[leg_name]
        end_effector = self._end_effectors[leg_name]

        self._move_group[leg_name].set_joint_value_target(pose, end_effector, True)

    def calculate_normal_trajectory(self, leg_name, subgait_name):
        """Calculate the pose of the non-capture-point leg and set this as the pose for this leg.

        :param leg_name: The name of the move group which does not use capture point
        :param subgait_name: the normal subgait name
        """
        side_prefix = 'right' if 'right' in leg_name else 'left'

        default_subgait = deepcopy(self.default_walk[subgait_name])

        non_capture_point_joints = []
        for joint in default_subgait.joints:
            if side_prefix in joint.name:
                non_capture_point_joints.append(joint)

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [joint.name for joint in non_capture_point_joints.joints]
        joint_state.position = [joint.setpoints[-1].position for joint in non_capture_point_joints]
        joint_state.velocity = [joint.setpoints[-1].velocity for joint in non_capture_point_joints]

        self._move_group[leg_name].set_joint_value_target(joint_state)

    @staticmethod
    def to_subgait(joints, duration, gait_name='balance_gait', gait_type='walk_like', version='moveit',
                   subgait_name='balance_subgait', description='Subgait created using the moveit motion planning.'):
        """Create a subgait using the standard format in the march shared classes."""
        return Subgait(joints, duration, gait_type, gait_name, subgait_name, version, description)

    def create_subgait_of_trajectory(self, joint_trajectory, subgait_name):
        """Create a subgait using the joint trajectory generated by the capture point pose.

        :param joint_trajectory: The capture point pose trajectory
        :param subgait_name: the normal subgait name

        :return: a populated subgait object generated from the joint trajectory
        """
        normal_subgait = deepcopy(self.default_walk[subgait_name])

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

        balance_subgait = BalanceGait.to_subgait(balance_joints, balance_duration, subgait_name=subgait_name)

        return balance_subgait

    def construct_subgait(self, capture_point_leg_name, subgait_name):
        """Construct a balance subgait.

        :param capture_point_leg_name: The name of the move group that should be used to create the balance subgait
        :param subgait_name: the normal subgait name

        :return: the balance subgait as a subgait object
        """
        non_capture_point_move_group = 'right_leg' if capture_point_leg_name == 'left_leg' else 'left_leg'

        self.calculate_capture_point_trajectory(capture_point_leg_name)
        self.calculate_normal_trajectory(non_capture_point_move_group, subgait_name)

        targets = \
            self._move_group['left_leg'].get_joint_value_target() + \
            self._move_group['right_leg'].get_joint_value_target()

        balance_subgait = self._move_group['all_legs'].plan(targets)
        balance_trajectory = balance_subgait.joint_trajectory

        if not balance_trajectory:
            rospy.logwarn('No valid balance trajectory for {ln} received from capture point leg, '
                          'returning default subgait'.format(ln=capture_point_leg_name))
            return self.default_walk[subgait_name]

        if not balance_trajectory.points:
            rospy.logwarn('Empty trajectory in {ln} received from capture point topic, '
                          'returning default subgait'.format(ln=capture_point_leg_name))
            return self.default_walk[subgait_name]

        balance_trajectory_subgait = self.create_subgait_of_trajectory(balance_trajectory, subgait_name)

        return balance_trajectory_subgait

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
