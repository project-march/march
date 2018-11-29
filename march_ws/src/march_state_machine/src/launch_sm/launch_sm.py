#!/usr/bin/env python
import smach

import hardware_sm.hardware_sm as hardware_sm
import config_sm.config_sm as config_sm
import simulation_sm.simulation_sm as simulation_sm

class DecisionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hardware', 'simulation'])

    def execute(self, userdata):
        return 'simulation'


def create():
    sm_launch = smach.StateMachine(outcomes=['succeeded', 'failed'])
    # Open the container
    with sm_launch:
        # Add states to the container
        smach.StateMachine.add('CONFIG', config_sm.create(),
                               transitions={'succeeded': 'RUNTIME-DECIDER',
                                            'failed': 'failed'})

        smach.StateMachine.add('RUNTIME-DECIDER', DecisionState(), transitions={'hardware': 'HARDWARE',
                                                                             'simulation': 'SIMULATION'})

        smach.StateMachine.add('HARDWARE', hardware_sm.create(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})
        smach.StateMachine.add('SIMULATION', simulation_sm.create(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})


    return sm_launch
