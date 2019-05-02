#!/usr/bin/env python

import unittest
import rosunit

from march_gait_selection.GaitSelection import GaitSelection
from march_shared_resources.msg import Subgait


class TestValidateVersionName(unittest.TestCase):

    gait_directory = "test/gaits_correct_walking_gait"

    actual_map_correct = \
        {
            "walking":
                {
                    "right_open": "test_a_bit_higher",
                    "left_swing": "test",
                    "right_close": "right_close"
                }
        }

    gait_selection = GaitSelection(gait_directory=gait_directory, gait_version_map=actual_map_correct)

    def test_validate_version_name_correct(self):
        self.assertTrue(self.gait_selection.validate_version_name("walking", "right_open", "test_a_bit_higher"))
        self.assertTrue(self.gait_selection.validate_version_name("walking", "right_close", "right_close"))
        self.assertTrue(self.gait_selection.validate_version_name("walking", "right_close", "not_the_default"))
        self.assertTrue(self.gait_selection.validate_version_name("walking", "left_swing", "test"))

    def test_validate_version_name_wrong_gait(self):
        self.assertFalse(self.gait_selection.validate_version_name("wrong_gait", "right_open", "right_open"))

    def test_validate_version_name_wrong_subgait(self):
        self.assertFalse(self.gait_selection.validate_version_name("walking", "wrong_subgait", "right_open"))

    def test_validate_version_name_wrong_version(self):
        self.assertFalse(self.gait_selection.validate_version_name("walking", "right_open", "wrong_version"))
