#!/usr/bin/env python
import smach

from Idle import Idle


def create():
    sm_ready = smach.StateMachine(outcomes=['succeeded', 'failed'])
    # Open the container
    with sm_ready:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    return sm_ready
