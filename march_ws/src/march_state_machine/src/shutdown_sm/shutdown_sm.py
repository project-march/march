#!/usr/bin/env python
import smach

from Shutdown import Shutdown


def create():
    sm_shutdown = smach.StateMachine(outcomes=['succeeded', 'failed'])
    # Open the container
    with sm_shutdown:
        # Add states to the container
        smach.StateMachine.add('SHUTDOWN', Shutdown(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    return sm_shutdown
