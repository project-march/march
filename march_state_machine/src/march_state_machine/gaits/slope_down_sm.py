import smach

from march_state_machine.state_machines.slope_state_machine import SlopeStateMachine
from march_state_machine.state_machines.step_state_machine import StepStateMachine
from march_state_machine.states.idle_state import IdleState


def create():
    sm_slope_down = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed', 'rejected'])
    sm_slope_down.register_io_keys(['sounds'])
    with sm_slope_down:
        smach.StateMachine.add('GAIT RD SLOPE DOWN', SlopeStateMachine('ramp_door_slope_down'),
                               transitions={'succeeded': 'STANDING SLOPE DOWN'})

        smach.StateMachine.add('GAIT RD SLOPE DOWN SINGLE', StepStateMachine('ramp_door_slope_down_single'),
                               transitions={'succeeded': 'STANDING SLOPE DOWN',
                                            'rejected': 'STANDING SLOPE DOWN'})

        smach.StateMachine.add('STANDING SLOPE DOWN', IdleState(gait_outcomes=['gait_ramp_door_slope_down_single',
                                                                               'gait_ramp_door_last_step']),
                               transitions={'gait_ramp_door_slope_down_single': 'GAIT RD SLOPE DOWN SINGLE',
                                            'gait_ramp_door_last_step': 'GAIT RD LAST STEP'})

        smach.StateMachine.add('GAIT RD LAST STEP', StepStateMachine('ramp_door_last_step'),
                               transitions={'rejected': 'STANDING SLOPE DOWN'})

    return sm_slope_down
