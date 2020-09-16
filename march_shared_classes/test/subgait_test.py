import unittest

import rospkg
from urdf_parser_py import urdf

from march_shared_classes.exceptions.gait_exceptions import NonValidGaitContent, SubgaitInterpolationError
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.limits import Limits
from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.gait.subgait import Subgait


class SubgaitTest(unittest.TestCase):
    def setUp(self):
        self.gait_name = 'walk'
        self.subgait_name = 'left_swing'
        self.version = 'MV_walk_leftswing_v2'
        self.resources_folder = rospkg.RosPack().get_path('march_shared_classes') + '/test/resources'
        self.robot = urdf.Robot.from_xml_file(rospkg.RosPack().get_path('march_description') + '/urdf/march4.urdf')
        self.subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                              gait=self.gait_name,
                                                                              subgait=self.subgait_name,
                                                                              version=self.version)
        self.subgait = Subgait.from_file(self.robot, self.subgait_path)

    # Subgait.from_file tests
    def test_from_file_valid_path(self):
        self.assertIsInstance(self.subgait, Subgait)

    def test_from_file_invalid_path(self):
        with self.assertRaises(FileNotFoundError):
            Subgait.from_file(self.robot, self.resources_folder + '/MV_walk_leftswing_v2.subgait')

    def test_from_file_none_path(self):
        with self.assertRaises(FileNotFoundError):
            Subgait.from_file(self.robot, None)

    def test_from_file_no_robot(self):
        self.assertIsNone(Subgait.from_file(None, self.subgait_path))

    # Subgait.from_files_interpolated tests
    def test_from_files_interpolated_correct(self):
        base_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                              gait=self.gait_name,
                                                                              subgait='left_close',
                                                                              version='MV_walk_leftclose_v1')
        other_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                               gait=self.gait_name,
                                                                               subgait='left_close',
                                                                               version='MV_walk_leftclose_v2')
        subgait = Subgait.from_files_interpolated(self.robot, base_subgait_path, other_subgait_path, 0.5)
        self.assertIsInstance(subgait, Subgait)

    # validate_subgait_transition tests
    def test_valid_subgait_transition(self):
        other_subgait_name = 'right_close'
        other_version = 'MV_walk_rightclose_v2'
        other_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                               gait=self.gait_name,
                                                                               subgait=other_subgait_name,
                                                                               version=other_version)
        other_subgait = Subgait.from_file(self.robot, other_subgait_path)
        self.assertTrue(self.subgait.validate_subgait_transition(other_subgait))

    def test_invalid_subgait_transition(self):
        other_subgait_name = 'right_close'
        other_version = 'MV_walk_rightclose_v2'
        other_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                               gait=self.gait_name,
                                                                               subgait=other_subgait_name,
                                                                               version=other_version)
        other_subgait = Subgait.from_file(self.robot, other_subgait_path)
        self.assertFalse(other_subgait.validate_subgait_transition(self.subgait))

    def test_invalid_subgait_transition_unequal_joints(self):
        other_subgait_name = 'right_close'
        other_version = 'MV_walk_rightclose_v2_seven_joints'
        other_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                               gait=self.gait_name,
                                                                               subgait=other_subgait_name,
                                                                               version=other_version)
        other_subgait = Subgait.from_file(self.robot, other_subgait_path)
        with self.assertRaises(NonValidGaitContent):
            other_subgait.validate_subgait_transition(self.subgait)

    # getters tests
    def test_get_unique_timestamps(self):
        self.assertEqual(len(self.subgait.get_unique_timestamps()), 9)

    def test_get_joint(self):
        self.assertIsInstance(self.subgait.get_joint('left_knee'), JointTrajectory)

    def test_get_joint_names(self):
        self.assertIsInstance(self.subgait.get_joint_names()[0], str)
        self.assertEqual(len(self.subgait.get_joint_names()), 8)

    def test_starting_position(self):
        position = 0.0
        subgait = Subgait([
            JointTrajectory('test', Limits(0.0, 0.0, 0.0), [
                Setpoint(0.0, position, 0.0),
                Setpoint(0.5, 1.0, 0.0),
            ], 1.0)], 1.0)
        self.assertDictEqual(subgait.starting_position, {'test': position})

    def test_final_position(self):
        position = 1.0
        subgait = Subgait([
            JointTrajectory('test', Limits(0.0, 0.0, 0.0), [
                Setpoint(0.0, 0.0, 0.0),
                Setpoint(0.5, position, 0.0),
            ], 1.0)], 1.0)
        self.assertDictEqual(subgait.final_position, {'test': position})

    def test_set_duration_with_scaling_smaller_duration(self):
        self.subgait.scale_timestamps_subgait(0.8)
        self.assertEqual(self.subgait.duration, 0.8)

    def test_set_duration_with_scaling_larger_duration(self):
        self.subgait.scale_timestamps_subgait(1.8)
        self.assertEqual(self.subgait.duration, 1.8)

    def test_set_duration_with_cut_off_instead_of_scaling(self):
        self.subgait.scale_timestamps_subgait(0.8, rescale=False)
        self.assertEqual(len(self.subgait.get_joint('left_knee')), 2)

    def test_equalize_amount_of_setpoints_with_higher_duration_new_gait(self):
        self.subgait.scale_timestamps_subgait(1.5)

        timestamps = sorted(set(self.subgait.get_unique_timestamps() + [1.1, 1.2, 1.3]))
        self.subgait.create_interpolated_setpoints([1.1, 1.2, 1.3])

        self.assertEqual(timestamps, self.subgait.get_unique_timestamps(),
                         msg='Scaling the function did not result in same timestamps and equal amount of setpoints'
                             '\nold timestamps: {old} \nnew timestamps: {new}'
                         .format(old=str(timestamps), new=str(self.subgait.get_unique_timestamps())))

    def test_equalize_amount_of_setpoints_with_lower_duration_new_gait(self):
        self.subgait.scale_timestamps_subgait(0.8)

        timestamps = sorted(set(self.subgait.get_unique_timestamps() + [0.6, 0.7, 0.75]))
        self.subgait.create_interpolated_setpoints([0.6, 0.7, 0.75])

        self.assertEqual(timestamps, self.subgait.get_unique_timestamps(),
                         msg='Scaling the function did not result in same timestamps and equal amount of setpoints'
                             '\nold timestamps: {old} \nnew timestamps: {new}'
                         .format(old=str(timestamps), new=str(self.subgait.get_unique_timestamps())))

    # Subgait.interpolate_subgaits tests
    def load_interpolatable_subgaits(self, subgait_name='left_close', base_version='MV_walk_leftclose_v1',
                                     other_version='MV_walk_leftclose_v2'):
        base_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                              gait=self.gait_name,
                                                                              subgait=subgait_name,
                                                                              version=base_version)
        base_subgait = Subgait.from_file(self.robot, base_subgait_path)
        other_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                               gait=self.gait_name,
                                                                               subgait=subgait_name,
                                                                               version=other_version)
        other_subgait = Subgait.from_file(self.robot, other_subgait_path)
        return base_subgait, other_subgait

    def test_interpolate_subgaits_wrong_parameter(self):
        # should be 0 <= parameter <= 1
        base_subgait, other_subgait = self.load_interpolatable_subgaits()
        with self.assertRaises(ValueError):
            Subgait.interpolate_subgaits(base_subgait, other_subgait, 2)

    def test_interpolate_subgaits_parameter_zero(self):
        base_subgait, other_subgait = self.load_interpolatable_subgaits()
        new_subgait = Subgait.interpolate_subgaits(base_subgait, other_subgait, 0)
        self.assertEqual(base_subgait, new_subgait)

    def test_interpolate_subgaits_parameter_one(self):
        base_subgait, other_subgait = self.load_interpolatable_subgaits()
        new_subgait = Subgait.interpolate_subgaits(base_subgait, other_subgait, 1)
        self.assertEqual(other_subgait, new_subgait)

    def test_interpolate_subgaits_interpolated(self):
        # test whether each setpoint is correctly interpolated
        parameter = 0.4
        base_subgait, other_subgait = self.load_interpolatable_subgaits()
        new_subgait = Subgait.interpolate_subgaits(base_subgait, other_subgait, parameter)
        for i, joint in enumerate(new_subgait.joints):
            for j, setpoint in enumerate(joint.setpoints):
                base_setpoint = base_subgait.joints[i].setpoints[j]
                other_setpoint = other_subgait.joints[i].setpoints[j]
                self.assertAlmostEqual(base_setpoint.time * parameter + (1 - parameter) * other_setpoint.time,
                                       setpoint.time, places=4)
                self.assertAlmostEqual(base_setpoint.position * parameter + (1 - parameter) * other_setpoint.position,
                                       setpoint.position, places=4)
                self.assertAlmostEqual(base_setpoint.velocity * parameter + (1 - parameter) * other_setpoint.velocity,
                                       setpoint.velocity, places=4)

    def test_interpolate_subgaits_wrong_amount_of_joints(self):
        base_subgait, other_subgait = self.load_interpolatable_subgaits('right_close', 'MV_walk_rightclose_v2',
                                                                        'MV_walk_rightclose_v2_seven_joints')
        with self.assertRaises(SubgaitInterpolationError):
            Subgait.interpolate_subgaits(base_subgait, other_subgait, 0.5)

    def test_interpolate_subgaits_wrong_joint_names(self):
        base_subgait, other_subgait = self.load_interpolatable_subgaits(
            other_version='MV_walk_leftclose_v2_wrong_joint_name')
        with self.assertRaises(SubgaitInterpolationError):
            Subgait.interpolate_subgaits(base_subgait, other_subgait, 0.5)

    def test_interpolate_subgaits_duration(self):
        base_subgait, other_subgait = self.load_interpolatable_subgaits()
        parameter = 0.2
        new_subgait = Subgait.interpolate_subgaits(base_subgait, other_subgait, parameter)
        new_duration = parameter * base_subgait.duration + (1 - parameter) * other_subgait.duration
        self.assertEqual(new_duration, new_subgait.duration)
