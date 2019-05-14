#!/usr/bin/env python
import unittest

import rospy

PKG = 'march_gait_selection'


class TestRosGaitSelection(unittest.TestCase):

    def test_gait_selection_services(self):
        rospy.init_node("test_gait_selection", anonymous=True, disable_signals=True)
        rospy.wait_for_service("march/gait_selection/get_version_map", 3)
        rospy.wait_for_service("march/gait_selection/get_directory_structure", 1)
        rospy.wait_for_service("march/gait_selection/set_version_map", 1)
        rospy.wait_for_service("march/gait_selection/set_version", 1)

        self.assertTrue(True)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_gait_selection', TestRosGaitSelection, ["--cov"])
