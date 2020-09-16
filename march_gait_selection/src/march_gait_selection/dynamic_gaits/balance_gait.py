from copy import deepcopy
import os
import sys

import moveit_commander
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_shared_resources.srv import CapturePointPose


class BalanceGait(GaitInterface):
    """Base class to create a gait using the moveit motion planning."""

    CAPTURE_POINT_SERVICE_TIMEOUT = 1

    def __init__(self, gait_name='balanced_walk', move_groups=None, default_walk=None):
        self.gait_name = gait_name
        self.move_group = move_groups

        self._default_walk = default_walk

        self._end_effectors = {'left_leg': 'foot_left', 'right_leg': 'foot_right'}
        self._capture_point_service = {'left_leg':
                                       rospy.ServiceProxy('/march/capture_point/foot_left', CapturePointPose),
                                       'right_leg':
                                       rospy.ServiceProxy('/march/capture_point/foot_right', CapturePointPose)}

        self._joint_state_target = JointState()
        self._joint_state_target.header = Header()
        self._joint_state_target.name = self.move_group['left_leg'].get_active_joints() +\
            self.move_group['right_leg'].get_active_joints()

        self._current_subgait = None
        self._current_subgait_duration = 0.0
        self._time_since_start = 0.0

    @classmethod
    def create_balance_subgait(cls, default_walk):
        """This class method should check if the balance variable is set and if so configure the motion planner.

        :param default_walk: gait object to base the balance gait on.
        """
        moveit_commander.roscpp_initialize(sys.argv)
        moveit_commander.RobotCommander()

        moveit_commander.PlanningSceneInterface()

        try:
            move_groups = {'all_legs': moveit_commander.MoveGroupCommander('all_legs'),
                           'left_leg': moveit_commander.MoveGroupCommander('left_leg'),
                           'right_leg': moveit_commander.MoveGroupCommander('right_leg')}

            for move_group in move_groups.values():
                move_group.set_pose_reference_frame('world')
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

    def set_swing_leg_target(self, leg_name, subgait_name):
        """Set the swing leg target to capture point.

        :param leg_name: The name of the used move group
        :param subgait_name: the normal subgait name
        """
        subgait_duration = self.default_walk[subgait_name].duration
        return_msg = self._capture_point_service[leg_name](duration=subgait_duration)

        if not return_msg.success:
            rospy.logwarn('No messages received from the capture point service.')
            return -1

        self.move_group[leg_name].set_joint_value_target(return_msg.capture_point, self._end_effectors[leg_name], True)

        return return_msg.duration

    def set_stance_leg_target(self, leg_name, subgait_name):
        """Set the target of the stance leg to the end of the gait file.

        :param leg_name: The name of the move group which does not use capture point
        :param subgait_name: the normal subgait name

        :return the duration of the default subgait
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

    def construct_trajectory(self, swing_leg, subgait_name):
        """Constructs a balance trajectory for all joints.

        :param capture_point_leg_name: The name of the move group that should be used to create the balance subgait
        :param subgait_name: the normal subgait name

        :return: the balance trajectory
        """
        stance_leg = 'right_leg' if swing_leg == 'left_leg' else 'left_leg'

        self.set_swing_leg_target(swing_leg, subgait_name)
        self.set_stance_leg_target(stance_leg, subgait_name)

        targets = \
            self.move_group['left_leg'].get_joint_value_target() + \
            self.move_group['right_leg'].get_joint_value_target()

        self._joint_state_target.header.stamp = rospy.Time.now()
        self._joint_state_target.position = targets

        balance_subgait = self.move_group['all_legs'].plan(self._joint_state_target)
        balance_trajectory = balance_subgait.joint_trajectory

        if not balance_trajectory:
            rospy.logwarn('No valid balance trajectory for {ln} received from capture point leg, '
                          'returning default subgait'.format(ln=swing_leg))
            return self.default_walk[subgait_name]

        if not balance_trajectory.points:
            rospy.logwarn('Empty trajectory in {ln} received from capture point topic, '
                          'returning default subgait'.format(ln=swing_leg))
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

    @staticmethod
    def export_to_file(subgait, gait_directory):
        """Write a subgait to a file for opening in the gait generator."""
        if gait_directory is None or gait_directory == '':
            return

        output_file_directory = os.path.join(gait_directory,
                                             subgait.gait_name.replace(' ', '_'),
                                             subgait.subgait_name.replace(' ', '_'))
        output_file_path = os.path.join(output_file_directory,
                                        subgait.version.replace(' ', '_') + '.subgait')

        rospy.loginfo('Writing gait to ' + output_file_path)

        if not os.path.isdir(output_file_directory):
            os.makedirs(output_file_directory)

        with open(output_file_path, 'w') as output_file:
            output_file.write(subgait.to_yaml())
