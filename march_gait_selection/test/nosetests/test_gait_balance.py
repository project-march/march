#!/usr/bin/env python

from copy import deepcopy
import unittest

from march_gait_selection.gait_selection import GaitSelection

PKG = 'march_gait_selection'
DIR = 'test/testing_gait_files'


class TestTransitionTrajectory(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._gait_selection = GaitSelection(PKG, DIR)

    def setUp(self):
        self.gait_selection = deepcopy(self._gait_selection)
