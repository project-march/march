#!/usr/bin/env python
import smach

from states.EmptyState import EmptyState

def create():
    sm_simulation = smach.Sequence(outcomes=['succeeded', 'failed'], connector_outcome='succeeded')
    # Open the container
    with sm_simulation:
        # Add states to the container
        smach.Sequence.add('GAZEBO', EmptyState())
        smach.Sequence.add('CONTROLLERS', EmptyState())

    return sm_simulation
