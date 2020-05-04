#!/usr/bin/env python

from copy import deepcopy
import unittest

from march_gait_selection.dynamic_gaits.balance_gait import BalanceGait
from march_gait_selection.gait_selection import GaitSelection
from march_shared_classes.exceptions.gait_exceptions import GaitError
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError, PackageNotFoundError
from march_shared_classes.gait.gait import Gait

valid_package = 'march_gait_selection'
valid_directory = 'test/testing_gait_files'


class TestGaitSelection(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._gait_selection = GaitSelection(valid_package, valid_directory)

    def setUp(self):
        self.gait_selection = deepcopy(self._gait_selection)
        self.gait_version_map = deepcopy(self._gait_selection.gait_version_map)

    # __init__ tests
    def test_init_with_wrong_package(self):
        with self.assertRaises(PackageNotFoundError):
            GaitSelection('wrong', valid_directory)

    def test_init_with_wrong_directory(self):
        with self.assertRaises(FileNotFoundError):
            GaitSelection(valid_package, 'wrong')

    # gait version map setter tests
    def test_set_gait_version_map_with_wrong_type(self):
        with self.assertRaises(TypeError):
            self.gait_selection.gait_version_map = 'wrong'

    def test_set_gait_version_map_with_empty_version_map(self):
        self.gait_version_map = {}
        with self.assertRaises(GaitError):
            self.gait_selection.gait_version_map = self.gait_version_map

    def test_set_gait_version_map_with_non_existing_gait_name(self):
        self.gait_version_map['wrong'] = self.gait_version_map['walk']
        with self.assertRaises(GaitError):
            self.gait_selection.gait_version_map = self.gait_version_map

    def test_set_gait_version_map_with_non_existing_subgait_name(self):
        self.gait_version_map['walk']['wrong'] = self.gait_version_map['walk']['right_open']
        with self.assertRaises(GaitError):
            self.gait_selection.gait_version_map = self.gait_version_map

    def test_set_gait_version_map_with_wrong_subgait_version(self):
        self.gait_version_map['walk']['right_open'] = 'wrong'
        with self.assertRaises(GaitError):
            self.gait_selection.gait_version_map = self.gait_version_map

    # load gaits tests
    def test_types_in_loaded_gaits(self):
        for gait in self.gait_selection.loaded_gaits:
            self.assertIsInstance(gait, (Gait, BalanceGait))

    # scan directory tests
    def test_scan_directory_top_level_length(self):
        directory = self.gait_selection.scan_directory()
        self.assertTrue(len(['stairs_up', 'walk', 'walk_medium', 'walk_small']), len(directory.keys()))

    def test_scan_directory_top_level_content(self):
        directory = self.gait_selection.scan_directory()
        self.assertTrue(all(key in ['stairs_up', 'walk', 'walk_medium', 'walk_small'] for key in directory.keys()))

    def test_scan_directory_subgait_versions(self):
        directory = self.gait_selection.scan_directory()
        walk_subgaits = directory['walk']['subgaits']
        self.assertEqual(walk_subgaits['left_swing'], ['MV_walk_leftswing_v2'])

    # get item tests
    def test_get_item_with_wrong_name(self):
        gait = self.gait_selection['wrong']
        self.assertEqual(gait, None)

    def test_get_item_valid_gait_name(self):
        gait = True if self.gait_selection['walk'] is not None else False
        self.assertTrue(gait)

    def test_get_item_type(self):
        gait = self.gait_selection['walk']
        self.assertIsInstance(gait, Gait)
