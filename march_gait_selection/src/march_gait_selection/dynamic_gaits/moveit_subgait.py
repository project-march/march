import sys

import moveit_commander
import rospy
from visualization_msgs.msg import Marker

from march_shared_resources.msg import Subgait


class MoveItSubgait(object):
    """Base class to create a subgait using the moveit motion planning."""

    def __init__(self):
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

        self._capture_point_msg = {'left_leg': None, 'right_leg': None}
        self._end_effectors = {'left_leg': 'left_foot', 'right_leg': 'right_foot'}

        rospy.Subscriber('/march/cp_marker_foot_left', Marker, queue_size=1, callback=self.capture_point_cb,
                         callback_args='left_leg')
        rospy.Subscriber('/march/cp_marker_foot_right', Marker, queue_size=1, callback=self.capture_point_cb,
                         callback_args='right_leg')

    def capture_point_cb(self, msg, leg_name):
        """Set latest message to variable.

        :param msg: The message from the capture point topic
        :param leg_name: The name of corresponding move group
        """
        self._capture_point_msg[leg_name] = msg

    def random_subgait(self):
        """Create random trajectory using the moveit motion planner.

        :return:
            A populated subgait message
        """
        self._move_group['left_leg'].set_random_target()
        trajectory_plan = self._move_group['left_leg'].plan()
        return self.to_subgait_msg_('Random moveit subgait', trajectory_plan.joint_trajectory)

    @staticmethod
    def to_subgait_msg_(name, trajectory, gait_type='walk_like', version='moveit',
                        description='Subgait created using the moveit motion planning.'):
        """Create a subgait message using the standard format in the march shared resources."""
        subgait_msg = Subgait()

        subgait_msg.name = name
        subgait_msg.description = description
        subgait_msg.trajectory = trajectory
        subgait_msg.version = version
        subgait_msg.gait_type = gait_type
        subgait_msg.duration = subgait_msg.trajectory.points[-1].time_from_start

        return subgait_msg
