#!/usr/bin/env python
import unittest

import rospy
import ast
from march_shared_resources.srv import StringTrigger, Trigger

PKG = 'march_gait_selection'


class TestRosGaitSelection(unittest.TestCase):

    def test_gait_selection_services(self):
        rospy.init_node("test_gait_selection", anonymous=True, disable_signals=True)
        rospy.wait_for_service("march/gait_selection/get_version_map", 3)
        rospy.wait_for_service("march/gait_selection/get_directory_structure", 1)
        rospy.wait_for_service("march/gait_selection/set_version_map", 1)
        rospy.wait_for_service("march/gait_selection/set_version", 1)

        self.assertTrue(True)

    def test_set_subgait_version(self):
        rospy.init_node("test_gait_selection", anonymous=True, disable_signals=True)

        # Initialize services
        rospy.wait_for_service('/march/gait_selection/get_version_map', 3)
        self.get_version_map = rospy.ServiceProxy('/march/gait_selection/get_version_map', Trigger)

        rospy.wait_for_service('/march/gait_selection/set_version_map', 3)
        self.set_version_map = rospy.ServiceProxy('/march/gait_selection/set_version_map', StringTrigger)

        # Set and get a mapping to see if it changes
        version_map_string = self.get_version_map().message
        version_map = ast.literal_eval(version_map_string)
        version_map["walk"]["right_close"] = 'not_the_default'

        result = self.set_version_map(str(version_map))
        self.assertTrue(result.success)
        new_version_map_string = self.get_version_map().message
        new_version_map = ast.literal_eval(new_version_map_string)
        self.assertEqual("not_the_default", new_version_map["walk"]["right_close"])

    def test_set_subgait_version_wrong(self):
        rospy.init_node("test_gait_selection", anonymous=True, disable_signals=True)

        # Initialize services
        rospy.wait_for_service('/march/gait_selection/get_version_map', 3)
        self.get_version_map = rospy.ServiceProxy('/march/gait_selection/get_version_map', Trigger)

        rospy.wait_for_service('/march/gait_selection/set_version_map', 3)
        self.set_version_map = rospy.ServiceProxy('/march/gait_selection/set_version_map', StringTrigger)

        # Set and get a mapping to see if it changes
        version_map_string = self.get_version_map().message
        version_map = ast.literal_eval(version_map_string)
        version_map["walk"]["right_close"] = 'wrong_subgait'

        result = self.set_version_map(str(version_map))
        self.assertFalse(result.success)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_gait_selection', TestRosGaitSelection)
