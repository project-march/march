import smach

from march_state_machine.state_machines.step_state_machine import StepStateMachine
from march_state_machine.states.idle_state import IdleState


def create():
    sm_tilted_path_sideways_end = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed', 'rejected'])
    sm_tilted_path_sideways_end.register_io_keys(['sounds'])
    with sm_tilted_path_sideways_end:
        smach.StateMachine.add('GAIT TP FIRST END', StepStateMachine('tilted_path_first_end',
                                                                     subgaits=['left_open', 'right_close']),
                               transitions={'succeeded': 'STANDING TP SIDEWAYS END'})

        smach.StateMachine.add('STANDING TP SIDEWAYS END', IdleState(gait_outcomes=['gait_tilted_path_second_end']),
                               transitions={'gait_tilted_path_second_end': 'GAIT TP SECOND END'})

        smach.StateMachine.add('GAIT TP SECOND END', StepStateMachine('tilted_path_second_end',
                                                                      subgaits=['left_open', 'right_close']),
                               transitions={'rejected': 'STANDING TP SIDEWAYS END'})

    return sm_tilted_path_sideways_end
