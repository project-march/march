#!/usr/bin/env python

from copy import deepcopy
import unittest

from march_gait_selection.dynamic_gaits.transition_subgait import TransitionSubgait
from march_gait_selection.gait_selection import GaitSelection
from march_shared_classes.exceptions.gait_exceptions import GaitError


VALID_PACKAGE = 'march_gait_selection'
VALID_DIRECTORY = 'test/testing_gait_files'


class TestTransitionTrajectory(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._gait_selection = GaitSelection(VALID_PACKAGE, VALID_DIRECTORY)

    def setUp(self):
        self.gait_selection = deepcopy(self._gait_selection)

    def test_invalid_old_gait_name(self):
        # check if wrong gait name causes right error
        with self.assertRaises(GaitError):
            TransitionSubgait.from_subgait_names(self.gait_selection, 'wrong', 'walk_medium', 'right_swing')

    def test_invalid_new_gait_name(self):
        # check if wrong gait name causes right error
        with self.assertRaises(GaitError):
            TransitionSubgait.from_subgait_names(self.gait_selection, 'walk_small', 'wrong', 'right_swing')

    def test_invalid_subgait_name(self):
        # check if wrong subgait name causes right error
        with self.assertRaises(GaitError):
            TransitionSubgait.from_subgait_names(self.gait_selection, 'walk_small', 'walk_medium', 'wrong')

    def test_invalid_gait_selection(self):
        # check if wrong gait selection module
        with self.assertRaises(GaitError):
            TransitionSubgait.from_subgait_names('wrong', 'walk_small', 'walk_medium', 'right_swing')

    def test_walk_transition_small_to_medium_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(self.gait_selection, 'walk_small', 'walk_medium', 'right_swing')

    def test_walk_transition_medium_to_small_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(self.gait_selection, 'walk_medium', 'walk_small', 'right_swing')

    def test_transition_walk_small_stairs_up_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(self.gait_selection, 'walk_small', 'stairs_up', 'right_swing')

    def test_transition_stairs_up_walk_small_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(self.gait_selection, 'stairs_up', 'walk_small', 'right_swing')

    def test_no_duplicate_duration_in_transition_subgait(self):
        #  Test if the TransitionSubgait is created without an error
        transition_subgait = TransitionSubgait.from_subgait_names(self.gait_selection, 'walk_small',
                                                                  'walk_medium', 'right_swing')

        for joint in transition_subgait.joints:
            durations = [setpoint.time for setpoint in joint.setpoints]

            self.assertLessEqual(len(durations), len(set(durations)), msg='Duplicate timestamps found in setpoints.')
