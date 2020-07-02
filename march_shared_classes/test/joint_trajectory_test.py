import unittest

from march_shared_classes.exceptions.gait_exceptions import SubgaitInterpolationError
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.limits import Limits
from march_shared_classes.gait.setpoint import Setpoint


class JointTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.joint_name = 'test_joint'
        self.limits = Limits(-1, 1, 2)
        self.duration = 2.0
        self.times = [0, self.duration / 2.0, self.duration]
        self.setpoints = [Setpoint(t, 2 * t, t / 2.0) for t in self.times]
        self.joint_trajectory = JointTrajectory(self.joint_name, self.limits, self.setpoints, self.duration)

    # get_setpoints_unzipped tests
    def test_get_setpoints_unzipped_time(self):
        output = self.joint_trajectory.get_setpoints_unzipped()
        self.assertEqual(output, (self.times, [2 * t for t in self.times], [t / 2.0 for t in self.times]))

    # set_duration tests
    def test_set_duration(self):
        expected = Setpoint(self.joint_trajectory.setpoints[-1].time * 2, self.joint_trajectory.setpoints[-1].position,
                            self.joint_trajectory.setpoints[-1].velocity / 2)
        self.joint_trajectory.set_duration(self.duration * 2)
        self.assertEqual(self.joint_trajectory.setpoints[-1], expected)

    # validate_joint_transition() tests
    def test_valid_joint_transition(self):
        next_setpoints = [Setpoint(t, 2 * (self.duration - t), (self.duration - t) / 2.0) for t in self.times]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertTrue(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_invalid_joint_transition_position(self):
        next_setpoints = [Setpoint(t, (self.duration - t), (self.duration - t) / 2.0) for t in self.times]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertFalse(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_invalid_joint_transition_velocity(self):
        next_setpoints = [Setpoint(t, 2 * (self.duration - t), (self.duration - t)) for t in self.times]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertFalse(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    # _validate_boundary_points tests
    def test_valid_boundary_points_nonzero_start_end_zero_speed(self):
        # First setpoint at t = 0.5 and last setpoint at t = 1.5 =/= duration have zero speed.
        setpoints = [Setpoint(0.5 * t + 0.5, (self.duration - t), t * 2 - t**2) for t in self.times]
        joint_trajectory = JointTrajectory(self.joint_name, self.limits, setpoints, self.duration)
        self.assertTrue(joint_trajectory._validate_boundary_points())

    def test_invalid_boundary_points_nonzero_start_nonzero_speed(self):
        # First setpoint at t = 1 has nonzero speed.
        setpoints = [Setpoint(0.5 * t + 1, (self.duration - t), (self.duration - t) * 2) for t in self.times]
        joint_trajectory = JointTrajectory(self.joint_name, self.limits, setpoints, self.duration)
        self.assertFalse(joint_trajectory._validate_boundary_points())

    def test_invalid_boundary_points_nonzero_end_nonzero_speed(self):
        # Last setpoint at t = 1 =/= duration has nonzero speed.
        setpoints = [Setpoint(0.5 * t, (self.duration - t), t / 2.0) for t in self.times]
        joint_trajectory = JointTrajectory(self.joint_name, self.limits, setpoints, self.duration)
        self.assertFalse(joint_trajectory._validate_boundary_points())

    # interpolate_setpoints tests
    def test_interpolation_start_point(self):
        interpolated_setpoint = self.joint_trajectory.get_interpolated_setpoint(0)
        self.assertEqual(interpolated_setpoint, self.setpoints[0])

    def test_interpolation_end_point(self):
        interpolated_setpoint = self.joint_trajectory.get_interpolated_setpoint(self.duration)
        self.assertEqual(interpolated_setpoint, self.setpoints[-1])

    def test_interpolation_mid_point_position(self):
        interpolated_setpoint = self.joint_trajectory.get_interpolated_setpoint(self.duration / 2)
        self.assertEqual(interpolated_setpoint, self.setpoints[1])

    def test_get_interpolated_setpoints_invalid_time_too_high(self):
        setpoint = self.joint_trajectory.get_interpolated_setpoint(self.duration + 1)
        self.assertEqual(setpoint, Setpoint(self.duration + 1, self.setpoints[-1].position, 0))

    def test_get_interpolated_setpoints_invalid_time_too_low(self):
        setpoint = self.joint_trajectory.get_interpolated_setpoint(-1)
        self.assertEqual(setpoint, Setpoint(-1, self.setpoints[0].position, 0))

    def test_get_interpolated_setpoints_home_subgait(self):
        self.joint_trajectory.setpoints = [Setpoint(3, 1, 1)]
        setpoint = self.joint_trajectory.get_interpolated_setpoint(1)
        self.assertEqual(setpoint, Setpoint(1, 1, 1))

    def test_interpolate_trajectories_unequal_limits(self):
        different_limits = Limits(-2, 3, 2)
        other_trajectory = JointTrajectory(self.joint_name, different_limits, self.setpoints, self.duration)
        with self.assertRaises(SubgaitInterpolationError):
            JointTrajectory.interpolate_joint_trajectories(self.joint_trajectory, other_trajectory, 0.5)

    def test_interpolate_trajectories_unequal_amount_setpoints(self):
        other_trajectory = JointTrajectory(self.joint_name, self.limits, [self.setpoints[0], self.setpoints[-1]],
                                           self.duration)
        with self.assertRaises(SubgaitInterpolationError):
            JointTrajectory.interpolate_joint_trajectories(self.joint_trajectory, other_trajectory, 0.5)

    def test_interpolate_trajectories_correct_duration(self):
        parameter = 0.5
        other_duration = self.duration + 1
        other_times = [0, other_duration / 2.0, other_duration]
        other_setpoints = [Setpoint(t, 2 * t, t / 2.0) for t in other_times]
        other_trajectory = JointTrajectory(self.joint_name, self.limits, other_setpoints,
                                           other_duration)
        new_trajectory = JointTrajectory.interpolate_joint_trajectories(self.joint_trajectory, other_trajectory,
                                                                        parameter)
        self.assertEqual(new_trajectory.duration, self.duration * parameter + (1 - parameter) * other_duration)
