#!/usr/bin/env python
import smach

from XmlState import XmlState
from UrdfState import UrdfState

def create():
    sm_config = smach.Sequence(outcomes=['succeeded', 'failed'], connector_outcome='succeeded')
    # Open the container
    with sm_config:
        # Add states to the container
        smach.StateMachine.add('XML', XmlState(),
                               transitions={'succeeded': 'URDF',
                                            'failed': 'failed'})
        smach.StateMachine.add('URDF', UrdfState(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    return sm_config
