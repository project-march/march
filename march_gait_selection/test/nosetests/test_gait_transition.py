#!/usr/bin/env python
import os
import unittest

import rospkg
from urdf_parser_py import urdf

from march_gait_selection.dynamic_gaits.transition_subgait import TransitionSubgait
from march_shared_classes.gait.subgait import Subgait

VALID_PACKAGE = 'march_gait_selection'
VALID_DIRECTORY = 'test/testing_gait_files'


class TestTransitionTrajectory(unittest.TestCase):

    def setUp(self):
        self.robot = urdf.Robot.from_xml_file(rospkg.RosPack().get_path('march_description') + '/urdf/march4.urdf')
        self.resources = os.path.join(rospkg.RosPack().get_path(VALID_PACKAGE), VALID_DIRECTORY)

    def test_walk_transition_small_to_medium_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        walk_small = Subgait.from_name_and_version(self.robot, self.resources, 'walk_small', 'right_swing', 'MIV_final')
        walk_medium = Subgait.from_name_and_version(self.robot, self.resources, 'walk_medium', 'right_swing',
                                                    'MV_walk_rightswing_v1')
        TransitionSubgait.from_subgaits(walk_small, walk_medium, 'test')

    def test_walk_transition_medium_to_small_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        walk_small = Subgait.from_name_and_version(self.robot, self.resources, 'walk_small', 'right_swing', 'MIV_final')
        walk_medium = Subgait.from_name_and_version(self.robot, self.resources, 'walk_medium', 'right_swing',
                                                    'MV_walk_rightswing_v1')
        TransitionSubgait.from_subgaits(walk_medium, walk_small, 'test')

    def test_transition_walk_small_stairs_up_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        walk_small = Subgait.from_name_and_version(self.robot, self.resources, 'walk_small', 'right_swing', 'MIV_final')
        stairs_up = Subgait.from_name_and_version(self.robot, self.resources, 'stairs_up', 'right_swing', 'MIV_final')
        TransitionSubgait.from_subgaits(walk_small, stairs_up, 'test')

    def test_transition_stairs_up_walk_small_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        walk_small = Subgait.from_name_and_version(self.robot, self.resources, 'walk_small', 'right_swing', 'MIV_final')
        stairs_up = Subgait.from_name_and_version(self.robot, self.resources, 'stairs_up', 'right_swing', 'MIV_final')
        TransitionSubgait.from_subgaits(stairs_up, walk_small, 'test')

    def test_no_duplicate_duration_in_transition_subgait(self):
        #  Test if the TransitionSubgait is created without an error
        walk_small = Subgait.from_name_and_version(self.robot, self.resources, 'walk_small', 'right_swing', 'MIV_final')
        walk_medium = Subgait.from_name_and_version(self.robot, self.resources, 'walk_medium', 'right_swing',
                                                    'MV_walk_rightswing_v1')
        transition_subgait = TransitionSubgait.from_subgaits(walk_small, walk_medium, 'test')

        for joint in transition_subgait.joints:
            durations = [setpoint.time for setpoint in joint.setpoints]

            self.assertLessEqual(len(durations), len(set(durations)), msg='Duplicate timestamps found in setpoints.')
