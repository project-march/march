#!/usr/bin/env python
import rosunit

from .test_gait_selection import TestGaitSelection
from .test_gait_transition import TestTransitionTrajectory

PKG = 'march_gait_selection'

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_gait_selection', TestGaitSelection)
    rosunit.unitrun(PKG, 'test_gait_transition', TestTransitionTrajectory)
