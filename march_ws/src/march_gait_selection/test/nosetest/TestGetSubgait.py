#!/usr/bin/env python

import unittest
import rosunit

from march_gait_selection.GaitSelection import GaitSelection
from march_shared_resources.msg import Subgait


class TestGetSubgait(unittest.TestCase):

    def setUp(self):
        self.gait_directory = "test/gaits_correct_walking_gait"

        self.actual_map_correct = \
            {
                "walking":
                    {
                        "right_open": "test_a_bit_higher",
                        "left_swing": "test",
                        "right_close": "right_close"
                    }
            }

        self.actual_map_wrong = \
            {
                "walking":
                    {
                        "right_open": "test_a_bit_higher",
                        "left_swing": "test",
                    }
            }

    def test_get_subgait_correct(self):
        gait_selection = GaitSelection(gait_directory=self.gait_directory, gait_version_map=self.actual_map_correct)
        subgait = gait_selection.get_subgait('walking', 'left_swing')
        self.assertEquals('left_swing', subgait.name)
        self.assertEquals('test', subgait.version)

    def test_get_subgait_wrong(self):
        gait_selection = GaitSelection(gait_directory=self.gait_directory, gait_version_map=self.actual_map_wrong)
        subgait = gait_selection.get_subgait('walking', 'right_close')
        self.assertEquals(Subgait(), subgait)

    def test_set_subgait(self):
        gait_selection = GaitSelection(gait_directory=self.gait_directory, gait_version_map=self.actual_map_correct)

        subgait = gait_selection.get_subgait('walking', 'right_close')

        self.assertEquals('right_close', subgait.version)
        gait_selection.set_subgait_version('walking', 'right_close', 'not_the_default')

        subgait = gait_selection.get_subgait('walking', 'right_close')
        self.assertEquals('not_the_default', subgait.version)

    def test_set_subgait_wrong(self):
        gait_selection = GaitSelection(gait_directory=self.gait_directory, gait_version_map=self.actual_map_correct)
        subgait = gait_selection.get_subgait('walking', 'right_close')
        self.assertEquals('right_close', subgait.version)
        gait_selection.set_subgait_version('walking', 'right_close', 'a_wrong_subgait')

        # Check the version hasn't changed
        subgait = gait_selection.get_subgait('walking', 'right_close')
        self.assertEquals('right_close', subgait.version)
