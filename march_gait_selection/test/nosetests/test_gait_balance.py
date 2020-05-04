#!/usr/bin/env python

from copy import deepcopy
import unittest

from march_gait_selection.dynamic_gaits.balance_gait import BalanceGait
from march_gait_selection.gait_selection import GaitSelection

PKG = 'march_gait_selection'
DIR = 'test/testing_gait_files'


class TestTransitionTrajectory(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._gait_selection = GaitSelection(PKG, DIR)

    def setUp(self):
        self.gait_selection = deepcopy(self._gait_selection)

    # __init__ tests
    def test_get_balance_walk_gait(self):
        is_balanced_walk = True if self.gait_selection['gait_balanced_walk'] is not None else False
        self.assertTrue(is_balanced_walk)

    def test_get_balance_walk_gait_type(self):
        balanced_walk = self.gait_selection['gait_balanced_walk']
        self.assertIsInstance(balanced_walk, BalanceGait)

    def test_initialised_move_groups(self):
        balanced_walk = self.gait_selection['gait_balanced_walk']
        has_move_groups = True if balanced_walk._move_groups is not None else False
        self.assertTrue(has_move_groups)
