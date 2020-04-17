import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros

from math import pi, asin, sqrt, acos

from march_shared_resources.msg import Subgait


class MoveItSubgait(object):
    def __init__(self):
        rospy.init_node('moveit_subgait', anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot_ = moveit_commander.RobotCommander()
        self.scene_ = moveit_commander.PlanningSceneInterface()

        self.move_group_right_ = moveit_commander.MoveGroupCommander("right_leg")
        self.move_group_left_ = moveit_commander.MoveGroupCommander("left_leg")

    def random_subgait(self):
        subgait = Subgait()

        self.move_group_left_.set_random_target()
        random_plan = self.move_group_left_.plan()

        subgait.trajectory = random_plan.joint_trajectory
        subgait.name = 'random'
        subgait.version = 'moveit'
        subgait.description = 'randomly generated subgait by MoveIt'
        subgait.gait_type = 'walk_like'

        return subgait






