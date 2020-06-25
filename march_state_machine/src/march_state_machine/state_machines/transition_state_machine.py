import ast

import rospy
import smach
from std_srvs.srv import Trigger

from march_state_machine.states.transition_state import TransitionState


class StateMachineWithTransition(smach.StateMachine):
    def __init__(self, transition_sequence, outcomes=None):
        self._default_outcomes = ['succeeded', 'preempted', 'failed', 'rejected']
        self._transition_sequence = transition_sequence

        if outcomes is not None:
            self._default_outcomes = outcomes

        super(StateMachineWithTransition, self).__init__(outcomes=self._default_outcomes, input_keys=['sounds'],
                                                         output_keys=['sounds'])

        with self.opened():
            smach.StateMachine.add('transition', TransitionState(transition_sequence),
                                   transitions={'aborted': 'failed'})

    def add(self, label, state, transitions=None, remapping=None):
        """Override the add-function to set the necessary values in the transition state within this state machine."""
        if transitions is None:
            transitions = {'succeeded': 'succeeded', 'failed': 'failed', 'transition': 'transition'}

        self._states['transition'].add_outcome(label)
        self._transitions['transition'][label] = label

        self.open()
        result = super(StateMachineWithTransition, self).add(label, state, transitions, remapping)
        self.close()

        return result

    def execute(self, parent_ud=smach.UserData()):
        """Reset the initial start state."""
        self.userdata.start_state = None

        try:
            rospy.wait_for_service('/march/gait_selection/get_version_map', 1)
            get_gait_version_map = rospy.ServiceProxy('/march/gait_selection/get_version_map', Trigger)

            gait_version_map = ast.literal_eval(get_gait_version_map().message)

            for label in self._states.copy():
                if label not in gait_version_map and label != 'transition':
                    rospy.logwarn('State {lb} does not have a valid gait file, '
                                  'removing state from transition state machine'.format(lb=label))

                    self._states['transition'].remove_outcome(label)

                    if label in self._states:
                        self._states.pop(label)

                    if label in self._transitions['transition']:
                        self._transitions['transition'].pop(label)

                    if label in self._transition_sequence:
                        self._transition_sequence.remove(label)

        except rospy.ROSException:
            rospy.logwarn('Transition state could not verify the transitions, do not use rocker-switch buttons!')
        except ValueError:
            pass

        if self.initial_state_label not in self._transition_sequence:
            return 'rejected'

        return super(StateMachineWithTransition, self).execute(parent_ud)

    @property
    def initial_state_label(self):
        return self._initial_state_label

    @initial_state_label.setter
    def initial_state_label(self, label):
        if label not in self._transition_sequence:
            rospy.logwarn('Requested starting label {label} is not in transition sequence {sequence}'
                          .format(label=label, sequence=self._transition_sequence))
            self._initial_state_label = None
        else:
            self._initial_state_label = label
