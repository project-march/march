#!/usr/bin/env python
from copy import deepcopy
import unittest

import rospkg
from urdf_parser_py import urdf

from march_gait_selection.gait_selection import GaitSelection
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError, PackageNotFoundError
from march_shared_classes.gait.gait import Gait

VALID_PACKAGE = 'march_gait_selection'
VALID_DIRECTORY = 'test/testing_gait_files'


class TestGaitSelection(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        robot = urdf.Robot.from_xml_file(rospkg.RosPack().get_path('march_description') + '/urdf/march4.urdf')
        cls._gait_selection = GaitSelection(VALID_PACKAGE, VALID_DIRECTORY, robot)

    def setUp(self):
        self.gait_selection = deepcopy(self._gait_selection)

    # __init__ tests
    def test_init_with_wrong_package(self):
        with self.assertRaises(PackageNotFoundError):
            GaitSelection('wrong', VALID_DIRECTORY, self.gait_selection.robot)

    def test_init_with_wrong_directory(self):
        with self.assertRaises(FileNotFoundError):
            GaitSelection(VALID_PACKAGE, 'wrong', self.gait_selection.robot)

    # load gaits tests
    def test_types_in_loaded_gaits(self):
        for gait in self.gait_selection._loaded_gaits.values():
            self.assertTrue(isinstance(gait, GaitInterface))

    def test_gait_selection_positions(self):
        self.assertIn('stand', self.gait_selection.positions)

    # scan directory tests
    def test_scan_directory_top_level_content(self):
        directory = self.gait_selection.scan_directory()
        directory_gaits = ['walk_medium', 'balance_walk', 'stairs_up', 'walk_small', 'walk']
        self.assertEqual(sorted(directory.keys()), sorted(directory_gaits))

    def test_scan_directory_subgait_versions(self):
        directory = self.gait_selection.scan_directory()
        self.assertEqual(directory['walk']['left_swing'], ['MV_walk_leftswing_v2'])

    # get item tests
    def test_get_item_with_wrong_name(self):
        self.assertIsNone(self.gait_selection['wrong'])

    def test_get_item_type(self):
        self.assertIsInstance(self.gait_selection['walk'], Gait)
