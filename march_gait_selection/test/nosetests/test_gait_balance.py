#!/usr/bin/env python

import unittest

from march_gait_selection.dynamic_gaits.balance_gait import BalanceGait
from march_gait_selection.gait_selection import GaitSelection
from march_shared_resources.msg import Subgait

VALID_PACKAGE = 'march_gait_selection'
VALID_DIRECTORY = 'test/testing_gait_files'


class TestBalanceGait(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.gait_selection = GaitSelection(VALID_PACKAGE, VALID_DIRECTORY)

    # __init__ tests
    def test_get_balance_walk_gait(self):
        self.assertIsNotNone(self.gait_selection['gait_balanced_walk'])

    def test_get_balance_walk_gait_type(self):
        self.assertIsInstance(self.gait_selection['gait_balanced_walk'], BalanceGait)

    def test_move_group_initiate(self):
        self.assertIsNotNone(self.gait_selection['gait_balanced_walk']._move_group)

    def test_random_subgait_message(self):
        random_moveit_subgait = self.gait_selection['gait_balanced_walk'].random_subgait()
        self.assertIsInstance(random_moveit_subgait, Subgait)
