#!/usr/bin/env python
import smach

from BatteryLow import BatteryLow


def create():
    sm_error = smach.StateMachine(outcomes=['succeeded', 'failed'])
    # Open the container
    with sm_error:
        # Add states to the container
        smach.StateMachine.add('BATTERY_LOW', BatteryLow(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    return sm_error
