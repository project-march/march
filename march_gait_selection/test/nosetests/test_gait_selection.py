#!/usr/bin/env python

from copy import deepcopy
import unittest

from march_gait_selection.dynamic_gaits.balance_gait import BalanceGait
from march_gait_selection.gait_selection import GaitSelection
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError, PackageNotFoundError
from march_shared_classes.gait.gait import Gait

VALID_PACKAGE = 'march_gait_selection'
VALID_DIRECTORY = 'test/testing_gait_files'


class TestGaitSelection(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._gait_selection = GaitSelection(VALID_PACKAGE, VALID_DIRECTORY)

    def setUp(self):
        self.gait_selection = deepcopy(self._gait_selection)

    # __init__ tests
    def test_init_with_wrong_package(self):
        with self.assertRaises(PackageNotFoundError):
            GaitSelection('wrong', VALID_DIRECTORY)

    def test_init_with_wrong_directory(self):
        with self.assertRaises(FileNotFoundError):
            GaitSelection(VALID_PACKAGE, 'wrong')

    # load gaits tests
    def test_types_in_loaded_gaits(self):
        for gait in self.gait_selection._loaded_gaits.values():
            self.assertTrue(isinstance(gait, Gait) or isinstance(gait, BalanceGait))

    # scan directory tests
    def test_scan_directory_top_level_content(self):
        directory = self.gait_selection.scan_directory()
        directory_gaits = ['walk_medium', 'balance_walk', 'stairs_up', 'walk_small', 'walk']
        self.assertTrue(all(gait in directory_gaits for gait in directory.keys()))

    def test_scan_directory_subgait_versions(self):
        directory = self.gait_selection.scan_directory()
        self.assertEqual(directory['walk']['left_swing'], ['MV_walk_leftswing_v2'])

    # get item tests
    def test_get_item_with_wrong_name(self):
        self.assertIsNone(self.gait_selection['wrong'])

    def test_get_item_valid_gait_name(self):
        self.assertIsNotNone(self.gait_selection['walk'])

    def test_get_item_type(self):
        self.assertIsInstance(self.gait_selection['walk'], Gait)
