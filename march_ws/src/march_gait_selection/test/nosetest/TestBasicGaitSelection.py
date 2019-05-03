#!/usr/bin/env python
import os
import unittest

import rospkg
import rosunit

from march_gait_selection.GaitSelection import GaitSelection

import TestGetSubgait
import TestValidateSubgaitName
import TestValidateVersionName
import TestValidateVersionMap
import TestValidateTrajectoryTransition

PKG = "march_gait_selection"


class TestBasicGaitSelection(unittest.TestCase):
    def test_gait_selection_creation_from_yaml(self):
        gait_selection = GaitSelection(default_yaml="/home/ishadijcks/march-iv/march_ws/src/march_gait_selection/test"
                                                    "/defaults/default_correct_walking_gait.yaml")
        actual_map = {
            "walking": {
                "right_open": "test_a_bit_higher",
                "left_swing": "test"
            }
        }

        self.assertEquals(actual_map, gait_selection.gait_version_map)
        self.assertEquals(gait_selection.gait_directory, os.path.join(
            rospkg.RosPack().get_path('march_gait_selection'), "test/gaits_correct_walking_gait"))

    def test_gait_selection_creation_parameters(self):
        gait_directory = "test/gaits_correct_walking_gait"

        actual_map = [{
            "walking": [{
                "right_open": "test_a_bit_higher"},
                {"left_swing": "test"}
            ]
        }]
        gait_selection = GaitSelection(gait_directory=gait_directory, gait_version_map=actual_map)

        self.assertEquals(actual_map, gait_selection.gait_version_map)
        self.assertEquals(gait_selection.gait_directory, os.path.join(
            rospkg.RosPack().get_path('march_gait_selection'), gait_directory))

    def test_gait_selection_compare_both_constructors(self):
        gait_directory = "test/gaits_correct_walking_gait"

        actual_map = {"walking": {
                "right_open": "test_a_bit_higher",
                "left_swing": "test"
            }
        }
        gait_selection_parameters = GaitSelection(gait_directory=gait_directory, gait_version_map=actual_map)

        gait_selection_yaml = GaitSelection(default_yaml="/home/ishadijcks/march-iv/march_ws/src/march_gait_selection"
                                                         "/test/defaults/default_correct_walking_gait.yaml")

        self.assertEquals(gait_selection_parameters.gait_directory, gait_selection_yaml.gait_directory)
        self.assertEquals(gait_selection_parameters.gait_version_map, gait_selection_yaml.gait_version_map)

    def test_gait_selection_empty(self):
        with self.assertRaises(Exception) as context:
            GaitSelection()

        self.assertTrue('Cannot instantiate GaitSelection without parameters, shutting down.' in context.exception)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_basic_gait_selection', TestBasicGaitSelection)
    rosunit.unitrun(PKG, 'test_get_subgait', TestGetSubgait)
    rosunit.unitrun(PKG, 'test_validate_subgait_name', TestValidateSubgaitName)
    rosunit.unitrun(PKG, 'test_validate_subgait_name', TestValidateVersionName)
    rosunit.unitrun(PKG, 'test_validate_version_name', TestValidateVersionMap)
    rosunit.unitrun(PKG, 'test_validate_trajectory_transition', TestValidateTrajectoryTransition)
