import moveit_commander
import rospy
import sys

from march_shared_resources.msg import Subgait
from visualization_msgs.msg import Marker


class MoveItSubgait(object):
    """Base class to create a subgait using the moveit motion planning"""
    def __init__(self):
        rospy.init_node('moveit_subgait', anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)
        moveit_commander.RobotCommander()
        moveit_commander.PlanningSceneInterface()

        self._move_group = {'left_leg': moveit_commander.MoveGroupCommander('left_leg'),
                            'right_leg': moveit_commander.MoveGroupCommander('right_leg')}

        self._capture_point_msg = {}

        rospy.Subscriber('/march/cp_marker_left_foot', Marker, queue_size=1, callback=self.capture_point_cb,
                         callback_args='left_leg')
        rospy.Subscriber('/march/cp_marker_right_foot', Marker, queue_size=1, callback=self.capture_point_cb,
                         callback_args='right_leg')

    def capture_point_cb(self, msg, leg_name):
        """ Set latest message to local variable

        :param msg:
        :param leg_name:
        """
        self._capture_point_msg[leg_name] = msg

    def create_subgait(self, leg_name):

        return self.to_subgait_msg_('', '')

    def random_subgait(self):
        """ Create random trajectory using the moveit motion planner

        :return:
        """
        self._move_group['left_leg'].set_random_target()
        trajectory_plan = self._move_group['left_leg'].plan()
        return self.to_subgait_msg_('Random moveit subgait', trajectory_plan)

    @staticmethod
    def to_subgait_msg_(name, trajectory, gait_type='walk_like', version='moveit',
                        description='Subgait created using the moveit motion planning.'):
        """Create a subgait message"""
        subgait_msg = Subgait()

        subgait_msg.name = name
        subgait_msg.description = description
        subgait_msg.trajectory = trajectory
        subgait_msg.version = version
        subgait_msg.gait_type = gait_type
        subgait_msg.duration = subgait_msg.trajectory.points[-1].time_from_start

        return subgait_msg