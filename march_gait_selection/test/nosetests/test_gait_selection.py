#!/usr/bin/env python

from copy import deepcopy
import unittest

from march_gait_selection.gait_selection import GaitSelection
from march_shared_classes.exceptions.gait_exceptions import GaitError
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError, PackageNotFoundError


valid_package = 'march_gait_selection'
valid_directory = 'test/testing_gait_files'

walk_medium = 'walk_medium'
walk_small = 'walk_small'

right_open = 'right_open'
right_swing = 'right_swing'
right_close = 'right_close'

wrong_name = 'wrong'


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

    # gait
