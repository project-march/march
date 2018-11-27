#!/usr/bin/env python
import smach

from ConfigState import ConfigState
from UrdfState import UrdfState


def create():
    sm_launch = smach.StateMachine(outcomes=['succeeded', 'failed'])
    # Open the container
    with sm_launch:
        # Add states to the container
        smach.StateMachine.add('CONFIG', ConfigState(),
                               transitions={'succeeded': 'URDF',
                                            'failed': 'failed'})
        smach.StateMachine.add('URDF', UrdfState(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    return sm_launch
