#!/usr/bin/env python

import unittest
import rosunit

from march_gait_selection.GaitSelection import GaitSelection
from march_shared_resources.msg import Subgait


class TestValidateVersionMap(unittest.TestCase):

    gait_directory = "test/gaits_correct_walking_gait"

    def test_validate_version_map_correct(self):
        map = \
            {
                "walking":
                    {
                        "right_open": "test_a_bit_higher",
                        "left_swing": "test",
                        "right_close": "right_close"
                    }
            }
        gait_selection = GaitSelection(gait_directory=self.gait_directory, gait_version_map=map)
        self.assertTrue(gait_selection.validate_version_map(map))

    def test_validate_version_map_wrong_gait(self):
        map = \
            {
                "walking":
                    {
                        "right_open": "test_a_bit_higher",
                        "left_swing": "test",
                        "right_close": "right_close"
                    },
                "wrong":
                    {
                        "right_open": "test_a_bit_higher",
                    }
            }
        gait_selection = GaitSelection(gait_directory=self.gait_directory, gait_version_map=map)
        self.assertFalse(gait_selection.validate_version_map(map))

    def test_validate_version_map_wrong_subgait(self):
        map = \
            {
                "walking":
                    {
                        "right_open": "test_a_bit_higher",
                        "wrong": "test",
                        "right_close": "right_close"
                    }
            }
        gait_selection = GaitSelection(gait_directory=self.gait_directory, gait_version_map=map)
        self.assertFalse(gait_selection.validate_version_map(map))

    def test_validate_version_map_wrong_version(self):
        map = \
            {
                "walking":
                    {
                        "right_open": "non_existant",
                        "left_swing": "test",
                        "right_close": "right_close"
                    }
            }
        gait_selection = GaitSelection(gait_directory=self.gait_directory, gait_version_map=map)
        self.assertFalse(gait_selection.validate_version_map(map))
