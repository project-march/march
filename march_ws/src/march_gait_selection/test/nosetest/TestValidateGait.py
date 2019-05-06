#!/usr/bin/env python

import unittest

import os
import rospkg
import yaml

from rospy_message_converter import message_converter

from march_gait_selection.GaitSelection import GaitSelection
from march_shared_resources.msg import Graph
from march_shared_resources.msg import GaitGoal


class TestValidateTrajectoryTransition(unittest.TestCase):

    def test_gait_correct(self):
        gait_selection = GaitSelection(default_yaml=os.path.join(rospkg.RosPack().get_path('march_gait_selection'),
                                                                 "test/defaults/default_correct_walking_gait.yaml"))
        gait_path = os.path.join(rospkg.RosPack().get_path('march_gait_selection'),
                                 "test/gaits_correct_walking_gait/walking/walking.gait")
        gait_yaml = yaml.load(open(gait_path), Loader=yaml.SafeLoader)
        print gait_yaml
        gait = message_converter.convert_dictionary_to_ros_message('march_shared_resources/GaitGoal',
                                                                   gait_yaml,
                                                                   kind="message")

        print gait
