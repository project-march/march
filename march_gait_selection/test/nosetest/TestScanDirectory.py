#!/usr/bin/env python

import os
import unittest
import rospkg

from march_gait_selection.GaitSelection import GaitSelection


class TestScanDirectory(unittest.TestCase):

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
    absolute_gait_directory = os.path.join(rospkg.RosPack().get_path('march_gait_selection'), gait_directory)

    def test_validate_scan_directory(self):
        self.maxDiff = None

        directory = {
            'walking': {
                'image':
                 self.absolute_gait_directory + '/walking/walking.png',
                'subgaits': {
                 'right_open': ['test_a_bit_higher'],
                 'right_close': ['not_the_default', 'right_close'],
                 'left_swing': ['test', 'incompatible_with_right_open_test_a_bit_higher']
                }
             }
        }
        self.assertEquals(directory, self.gait_selection.scan_directory())
