#!/usr/bin/env python
import smach

from states.EmptyState import EmptyState

def create():
    sm_hardware = smach.Sequence(outcomes=['succeeded', 'failed'], connector_outcome='succeeded')
    # Open the container
    with sm_hardware:
        # Add states to the container
        smach.Sequence.add('Ethercat', EmptyState())
        smach.Sequence.add('Sensors', EmptyState())
        smach.Sequence.add('BMS', EmptyState())

    return sm_hardware
