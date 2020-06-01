import smach

from march_state_machine.state_machines.step_state_machine import StepStateMachine
from march_state_machine.states.idle_state import IdleState


def create():
    sm_tilted_path_left_flexed_knee_step = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed', 'rejected'])
    sm_tilted_path_left_flexed_knee_step.register_io_keys(['sounds'])
    with sm_tilted_path_left_flexed_knee_step:
        smach.StateMachine.add('GAIT TP LEFT FLEXED KNEE STEP', StepStateMachine('tilted_path_left_flexed_knee_step',
                                                                                 subgaits=['right_open', 'left_close']),
                               transitions={'succeeded': 'STANDING TP LEFT STRAIGHT'})

        smach.StateMachine.add('GAIT TP LEFT SINGLE STEP', StepStateMachine('tilted_path_left_single_step',
                                                                            subgaits=['right_open', 'left_close']),
                               transitions={'succeeded': 'STANDING TP LEFT STRAIGHT',
                                            'rejected': 'STANDING TP LEFT STRAIGHT'})

        smach.StateMachine.add('STANDING TP LEFT STRAIGHT',
                               IdleState(gait_outcomes=['gait_tilted_path_left_single_step',
                                                        'gait_tilted_path_left_straight_end']),
                               transitions={'gait_tilted_path_left_single_step': 'GAIT TP LEFT SINGLE STEP',
                                            'gait_tilted_path_left_straight_end': 'GAIT TP LEFT STRAIGHT END'})

        smach.StateMachine.add('GAIT TP LEFT STRAIGHT END', StepStateMachine('tilted_path_left_straight_end',
                                                                             subgaits=['left_open', 'right_close']),
                               transitions={'rejected': 'STANDING TP LEFT STRAIGHT'})

    return sm_tilted_path_left_flexed_knee_step
