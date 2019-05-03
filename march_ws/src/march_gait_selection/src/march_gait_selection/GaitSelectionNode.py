#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib
from std_srvs.srv import Trigger
from march_shared_resources.srv import StringTrigger

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


def set_selected_versions_callback(msg, gait_selection):
    try:
        string = msg.string.replace(".subgait", "")
        gait_name, subgait_name, version_name = string.split("/")
    except ValueError:
        return [False, "Could not split gait " + msg.string + "."]

    if gait_selection.set_subgait_version(gait_name, subgait_name, version_name):
        return [True, "Subgait " + gait_name + "/" + subgait_name + " now uses version " + version_name + "."]
    return [False, "Version " + gait_name + "/" + subgait_name + "/" + version_name + ".subgait is not valid."]


if __name__ == '__main__':
    rospy.init_node("gait_selection")

    default_yaml = os.path.join(rospkg.RosPack().get_path('march_gait_selection'), 'gait', 'default.yaml')
    gait_selection = GaitSelection(default_yaml=default_yaml)

    # Use lambdas to process service calls inline
    get_gait_directory_service = rospy.Service('get_gait_directory', Trigger,
                                               lambda msg: [True, gait_selection.gait_directory])
    get_gait_version_map_service = rospy.Service('get_gait_version_map', Trigger,
                                                 lambda msg: [True, str(gait_selection.gait_version_map)])
    get_all_gait_files_service = rospy.Service('get_all_gait_files', Trigger,
                                               lambda msg: [True, str(gait_selection.scan_directory())])
    set_selected_versions_service = rospy.Service('set_selected_versions', StringTrigger,
                                                  lambda msg: set_selected_versions_callback(msg, gait_selection))

    perform_gait_server = actionlib.SimpleActionServer("march/gait/perform", GaitNameAction,
                                                       execute_cb=target_gait_callback,
                                                       auto_start=False)
    perform_gait_server.start()

    schedule_gait_client = actionlib.SimpleActionClient("march/gait/schedule", GaitAction)
    schedule_gait_client.wait_for_server()

    rate = rospy.Rate(10)
    rospy.spin()
