#!/usr/bin/env python

import unittest

from march_gait_selection.dynamic_gaits.balance_gait import BalanceGait

PKG = 'march_gait_selection'

VALID_PACKAGE = 'march_gait_selection'
VALID_DIRECTORY = 'test/testing_gait_files'


class TestBalanceGait(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.balance_gait = BalanceGait.create_balance_subgait(None)

    def test_move_group_initiate(self):
        self.assertIsNotNone(self.balance_gait.move_group)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_balance_gait', TestBalanceGait)
