#!/usr/bin/env python

from copy import deepcopy
import unittest

from march_gait_selection.dynamic_gaits.balance_gait import BalanceGait
from march_gait_selection.gait_selection import GaitSelection

VALID_PACKAGE = 'march_gait_selection'
VALID_DIRECTORY = 'test/testing_gait_files'


class TestBalanceGait(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._gait_selection = GaitSelection(VALID_PACKAGE, VALID_DIRECTORY)

    def setUp(self):
        self.gait_selection = deepcopy(self._gait_selection)

    # __init__ tests
    def test_get_balance_walk_gait(self):
        self.assertFalse(self.gait_selection['gait_balanced_walk'] is None)

    def test_get_balance_walk_gait_type(self):
        balanced_walk = self.gait_selection['gait_balanced_walk']
        self.assertIsInstance(balanced_walk, BalanceGait)
