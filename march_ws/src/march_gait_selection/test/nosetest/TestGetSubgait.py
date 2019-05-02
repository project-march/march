#!/usr/bin/env python

import unittest
import rosunit

from march_gait_selection.GaitSelection import GaitSelection


class TestGetSubgait(unittest.TestCase):

    def test_get_subgait(self):
        gait_directory = "test/gaits_correct_walking_gait"

        actual_map =\
            {
            "walking":
                {
                    "right_open": "test_a_bit_higer",
                    "left_swing": "test"
                }
            }
        gait_selection = GaitSelection(gait_directory=gait_directory, gait_version_map=actual_map)

        self.assertEquals('test', gait_selection.get_subgait('walking', 'left_swing'))
