import sys

import moveit_commander
import rospy
from visualization_msgs.msg import Marker

from march_shared_resources.msg import Subgait


class BalanceGait(object):
    """Base class to create a gait using the moveit motion planning."""

    def __init__(self, gait_name='balanced_walk'):
        self.gait_name = gait_name
        self.use_balance = rospy.get_param('balance')

        if self.use_balance:
            rospy.init_node('moveit_subgait', anonymous=True)

            moveit_commander.roscpp_initialize(sys.argv)
            moveit_commander.RobotCommander()
            moveit_commander.PlanningSceneInterface()

            try:
                self._move_group = {'left_leg': moveit_commander.MoveGroupCommander('left_leg'),
                                    'right_leg': moveit_commander.MoveGroupCommander('right_leg')}
            except RuntimeError:
                rospy.logerr('Could not connect to move groups, aborting initialisation of the moveit subgait class')
                return

        self._end_effectors = {'left_leg': 'left_foot', 'right_leg': 'right_foot'}
        self._latest_capture_point_msg_time = {'left_leg': None, 'right_leg': None}
        self._capture_point_position = {'left_leg': None, 'right_leg': None}

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
        self._capture_point_position[leg_name] = msg.pose.position

    def calculate_trajectory(self, leg_name):
        """Calculate the trajectory using moveit and return as a subgait msg format."""
        return self.random_subgait()

    def random_subgait(self):
        """Create random trajectory using the moveit motion planner.

        :return:
            A populated subgait message
        """
        self._move_group['left_leg'].set_random_target()
        trajectory_msg = self._move_group['left_leg'].plan()
        return self.to_subgait_msg_('Random moveit subgait', trajectory_msg.joint_trajectory)

    @staticmethod
    def to_subgait_msg_(name, trajectory_msg, gait_type='walk_like', version='moveit',
                        description='Subgait created using the moveit motion planning.'):
        """Create a subgait message using the standard format in the march shared resources."""
        subgait_msg = Subgait()

        subgait_msg.name = name
        subgait_msg.description = description
        subgait_msg.trajectory = trajectory_msg
        subgait_msg.version = version
        subgait_msg.gait_type = gait_type
        subgait_msg.duration = subgait_msg.trajectory.points[-1].time_from_start

        return subgait_msg

    def __getitem__(self, name):
        """Return the trajectory of a move group based on capture point in subgait msg format."""
        if name[0:4] == 'right':
            return self.calculate_trajectory('right_leg')
        elif name[0:5] == 'left':
            return self.calculate_trajectory('left_leg')
        else:
            return None
