import smach

from march_state_machine.state_machines.step_state_machine import StepStateMachine
from march_state_machine.states.idle_state import IdleState


def create():
    sm_tilted_path_right_knee_bend = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed', 'rejected'])
    sm_tilted_path_right_knee_bend.register_io_keys(['sounds'])
    with sm_tilted_path_right_knee_bend:
        smach.StateMachine.add('GAIT TP RIGHT KNEE BEND', StepStateMachine('tilted_path_right_knee_bend',
                                                                           subgaits=['right_open']),
                               transitions={'succeeded': 'STANDING TP RIGHT STRAIGHT'})

        smach.StateMachine.add('GAIT TP RIGHT SINGLE STEP', StepStateMachine('tilted_path_right_single_step',
                                                                             subgaits=['left_open', 'right_close']),
                               transitions={'succeeded': 'STANDING TP RIGHT STRAIGHT',
                                            'rejected': 'STANDING TP RIGHT STRAIGHT'})

        smach.StateMachine.add('STANDING TP RIGHT STRAIGHT',
                               IdleState(gait_outcomes=['gait_tilted_path_right_single_step',
                                                        'gait_tilted_path_right_straight_end']),
                               transitions={'gait_tilted_path_right_single_step': 'GAIT TP RIGHT SINGLE STEP',
                                            'gait_tilted_path_right_straight_end': 'GAIT TP RIGHT STRAIGHT END'})

        smach.StateMachine.add('GAIT TP RIGHT STRAIGHT END', StepStateMachine('tilted_path_right_straight_end',
                                                                              subgaits=['right_open', 'left_close']),
                               transitions={'rejected': 'STANDING TP RIGHT STRAIGHT'})

    return sm_tilted_path_right_knee_bend
