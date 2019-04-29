#!/usr/bin/env python
import unittest

import rospy

PKG = 'march_gait_selection'


class TestStateMachine(unittest.TestCase):

    """Test cases for the gait selection package @TODO(Isha, Tim) Create tests"""
    def test_gait_selection(self):
        rospy.init_node("test_gait_selection", anonymous=True, disable_signals=True)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_state_machine', TestStateMachine)
