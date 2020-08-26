from copy import deepcopy
import sys

import moveit_commander
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from march_gait_selection.state_machine.gait_interface import GaitInterface


class BalanceGait(GaitInterface):
    """Base class to create a gait using the moveit motion planning."""

    def __init__(self, gait_name='balanced_walk', move_groups=None, default_walk=None):
        self.gait_name = gait_name
        self.move_group = move_groups

        self._default_walk = default_walk

        self._end_effectors = {'left_leg': 'foot_left', 'right_leg': 'foot_right'}
        self._capture_point_pose = {'left_leg': None, 'right_leg': None}

        self._latest_capture_point_msg_time = {'left_leg': None, 'right_leg': None}
        self._old_capture_point_msg_time = {'left_leg': None, 'right_leg': None}

        self._current_subgait = None
        self._current_subgait_duration = 0.0
        self._time_since_start = 0.0

        rospy.Subscriber('/march/cp_marker_foot_left', Marker, queue_size=1, callback=self.capture_point_cb,
                         callback_args='left_leg')
        rospy.Subscriber('/march/cp_marker_foot_right', Marker, queue_size=1, callback=self.capture_point_cb,
                         callback_args='right_leg')

    @classmethod
    def create_balance_subgait(cls, default_walk):
        """This class method should check if the balance variable is set and if so configure the motion planner.

        :param default_walk: gait object to base the balance gait on.
        """
        if not rospy.get_param('/balance', False):
            return None

        moveit_commander.roscpp_initialize(sys.argv)
        moveit_commander.RobotCommander()

        moveit_commander.PlanningSceneInterface()

        try:
            move_groups = {'all_legs': moveit_commander.MoveGroupCommander('all_legs'),
                           'left_leg': moveit_commander.MoveGroupCommander('left_leg'),
                           'right_leg': moveit_commander.MoveGroupCommander('right_leg')}
        except RuntimeError:
            rospy.logerr('Could not connect to move groups, aborting initialisation of the moveit subgait class')
            return None

        return cls(move_groups=move_groups, default_walk=default_walk)

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

        self.move_group[leg_name].set_joint_value_target(pose, end_effector, True)

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
        joint_state.name = [joint.name for joint in non_capture_point_joints]
        joint_state.position = [joint.setpoints[-1].position for joint in non_capture_point_joints]
        joint_state.velocity = [joint.setpoints[-1].velocity for joint in non_capture_point_joints]

        self.move_group[leg_name].set_joint_value_target(joint_state)

    def construct_trajectory(self, capture_point_leg_name, subgait_name):
        """Constructs a balance trajectory for all joints.

        :param capture_point_leg_name: The name of the move group that should be used to create the balance subgait
        :param subgait_name: the normal subgait name

        :return: the balance trajectory
        """
        non_capture_point_move_group = 'right_leg' if capture_point_leg_name == 'left_leg' else 'left_leg'

        self.calculate_capture_point_trajectory(capture_point_leg_name)
        self.calculate_normal_trajectory(non_capture_point_move_group, subgait_name)

        targets = \
            self.move_group['left_leg'].get_joint_value_target() + \
            self.move_group['right_leg'].get_joint_value_target()

        balance_subgait = self.move_group['all_legs'].plan(targets)
        balance_trajectory = balance_subgait.joint_trajectory

        if not balance_trajectory:
            rospy.logwarn('No valid balance trajectory for {ln} received from capture point leg, '
                          'returning default subgait'.format(ln=capture_point_leg_name))
            return self.default_walk[subgait_name]

        if not balance_trajectory.points:
            rospy.logwarn('Empty trajectory in {ln} received from capture point topic, '
                          'returning default subgait'.format(ln=capture_point_leg_name))
            return self.default_walk[subgait_name]

        return balance_trajectory

    def get_joint_trajectory_msg(self, name):
        """Returns the trajectory of a subgait name that could use moveit.

        :param name: the name of the subgait
        """
        if name == 'right_open_2':
            return self.construct_trajectory('right_leg', name)
        elif name == 'left_swing_2':
            return self.construct_trajectory('left_leg', name)
        elif name == 'right_swing_2':
            return self.construct_trajectory('right_leg', name)
        else:
            return self.default_walk[name].to_joint_trajectory_msg()

    # GaitInterface
    @property
    def name(self):
        return self.gait_name

    @property
    def subgait_name(self):
        return self._current_subgait

    @property
    def duration(self):
        return self._current_subgait_duration

    @property
    def gait_type(self):
        return 'walk_like'

    @property
    def starting_position(self):
        return self._default_walk.starting_position

    @property
    def final_position(self):
        return self._default_walk.final_position

    def start(self):
        self._current_subgait = self._default_walk.graph.start_subgaits()[0]
        self._time_since_start = 0.0
        trajectory = self.get_joint_trajectory_msg(self._current_subgait)
        self._current_subgait_duration = trajectory.points[-1].time_from_start.to_sec()
        return trajectory

    def update(self, elapsed_time):
        self._time_since_start += elapsed_time
        if self._time_since_start < self._current_subgait_duration:
            return None, False
        else:
            next_subgait = self._default_walk.graph[(self._current_subgait, self._default_walk.graph.TO)]

            if next_subgait == self._default_walk.graph.END:
                return None, True

            trajectory = self.get_joint_trajectory_msg(next_subgait)
            self._current_subgait = next_subgait
            self._current_subgait_duration = trajectory.points[-1].time_from_start.to_sec()
            self._time_since_start = 0.0
            return trajectory, False

    def end(self):
        self._current_subgait = None
        self._current_subgait_duration = 0.0
