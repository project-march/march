import os

import rospy
import rospkg
import actionlib

from trajectory_msgs.msg import JointTrajectoryPoint

from march_shared_resources.msg import GaitNameAction, GaitAction, GaitActionGoal, GaitGoal

from GaitSelection import GaitSelection


def target_gait_callback(self, goal):
    rospy.loginfo(" %s pose requested", goal.subgait_name)
    # trajectory_result = self.schedule_gait(goal.subgait_name)


def schedule_gait(self, gait_name, subgait):
    gait_action_goal = GaitGoal()
    gait_action_goal.name = gait_name
    gait_action_goal.current_subgait = subgait

    self.schedule_gait_client.send_goal(gait_action_goal)

    self.schedule_gait_client.wait_for_result()
    return self.schedule_gait_client.get_result()


if __name__ == '__main__':
    rospy.init_node("gait_selection")

    default_yaml = os.path.join(rospkg.RosPack().get_path('march_gait_selection'), 'gait', 'default.yaml')
    GaitSelection = GaitSelection(default_yaml)

    perform_gait_server = actionlib.SimpleActionServer("march/gait/perform", GaitNameAction,
                                                       execute_cb=target_gait_callback,
                                                       auto_start=False)
    perform_gait_server.start()

    schedule_gait_client = actionlib.SimpleActionClient("march/gait/schedule", GaitAction)
    schedule_gait_client.wait_for_server()

    rate = rospy.Rate(10)
    rospy.spin()
