import sys

import moveit_commander
import rospy

from march_shared_resources.msg import Subgait


class MoveItSubgait(object):

    def __init__(self):
        rospy.init_node('moveit_subgait', anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)

        moveit_commander.RobotCommander()
        moveit_commander.PlanningSceneInterface()

        self.move_group_right_ = moveit_commander.MoveGroupCommander('right_leg')
        self.move_group_left_ = moveit_commander.MoveGroupCommander('left_leg')

    def random_subgait(self):
        subgait = Subgait()

        self.move_group_left_.set_random_target()
        random_plan = self.move_group_left_.plan()

        subgait.trajectory = random_plan.joint_trajectory
        subgait.name = 'random'
        subgait.version = 'moveit'
        subgait.description = 'randomly generated subgait by MoveIt'
        subgait.gait_type = 'walk_like'
        subgait.duration = subgait.trajectory.points[-1].time_from_start

        return subgait
