import threading

import rospy
import smach

from march_state_machine.control_flow import control_flow


class IdleState(smach.State):
    """State in which the exoskeleton is not moving.

    Listens to instructions from the input device and reacts if they are known transitions.
    """

    def __init__(self, gait_outcomes):
        outcomes = ['failed', 'preempted'] + gait_outcomes
        super(IdleState, self).__init__(outcomes)

        self._is_balance_used = rospy.get_param('/balance', False)
        self._result_gait = None
        self._trigger_event = threading.Event()

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        self._trigger_event.clear()
        self._result_gait = None

        control_flow.reset_gait()
        control_flow.reset_stop()

        control_flow.set_stopped_callback(self._stopped_cb)
        control_flow.set_gait_transition_callback(self._transition_cb)
        control_flow.set_gait_selected_callback(self._gait_cb)
        control_flow.set_state_machine_to_unknown(self._return_failed)

        self._trigger_event.wait()
        control_flow.clear_callbacks()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self._result_gait:
            return self._result_gait

        return 'failed'

    @staticmethod
    def _stopped_cb():
        rospy.logwarn('Idle state does not respond to stop')
        control_flow.reset_stop()

    @staticmethod
    def _transition_cb():
        rospy.logwarn('Idle state does not respond to transition input')
        control_flow.reset_transition()

    def _gait_cb(self, gait):
        if gait in self.get_registered_outcomes():
            if gait == 'gait_balanced_walk':
                if not self._is_balance_used:
                    rospy.logwarn('Balance cannot be used, MoveIt! is not launched')
                    control_flow.gait_rejected()
                    return

            rospy.logdebug('Accepted {0}'.format(gait))
            self._result_gait = gait
            control_flow.gait_accepted()
            self._trigger_event.set()
        else:
            rospy.logwarn('The {0} is not a possible gait in the current state'.format(gait))
            control_flow.gait_rejected()

    def request_preempt(self):
        super(IdleState, self).request_preempt()
        self._trigger_event.set()

    def _return_failed(self):
        rospy.logwarn('Current state is set to unknown.')
        self._result_gait = None
        self._trigger_event.set()
