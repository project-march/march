from enum import Enum

import rospy

from march_shared_resources.msg import GaitInstruction, GaitInstructionResponse


class TransitionRequest(Enum):
    NONE = 0
    DECREASE_SIZE = -1
    INCREASE_SIZE = 1


class StateMachineInput(object):

    def __init__(self):
        self._stopped = False
        self._paused = False
        self._unknown = False
        self._transition_index = 0
        self._gait = None

        self._instruction_subscriber = rospy.Subscriber('/march/input_device/instruction',
                                                        GaitInstruction,
                                                        self._callback_input_device_instruction)
        self._instruction_response_publisher = rospy.Publisher('/march/input_device/instruction_response',
                                                               GaitInstructionResponse,
                                                               queue_size=20)

    def get_transition_request(self):
        """Used to return the transition request as an enum.

        :returns TransitionRequest
        """
        if self._transition_index == GaitInstruction.INCREMENT_STEP_SIZE:
            return TransitionRequest.INCREASE_SIZE

        if self._transition_index == GaitInstruction.DECREMENT_STEP_SIZE:
            return TransitionRequest.DECREASE_SIZE

        return TransitionRequest.NONE

    def stop_requested(self):
        """Returns True when the current gait should stop, otherwise False."""
        return self._stopped

    def pause_requested(self):
        """Returns True when the input requests to pause the current gait, otherwise False."""
        return self._paused

    def unknown_requested(self):
        """Returns True when the input requests to transition to the UNKNOWN state, otherwise False."""
        return self._unknown

    def transition_requested(self):
        """Returns True when the input requests a gait transition, otherwise False."""
        return self._transition_index != 0

    def gait_requested(self):
        """Returns True when the input has a gait selected, otherwise False."""
        return self._gait is not None

    def gait_name(self):
        """Returns a name of the gait that has been selected, if one was selected, otherwise None."""
        return self._gait

    def reset(self):
        """Resets the input state to its original state on init."""
        self._stopped = False
        self._paused = False
        self._unknown = False
        self._transition_index = 0
        self._gait = None

    def stop_accepted(self):
        self._stopped = False

    def stop_rejected(self):
        self._stopped = False

    def gait_accepted(self):
        """Callback called when the state machine accepts the requested gait."""
        response = GaitInstructionResponse(result=GaitInstructionResponse.GAIT_ACCEPTED)
        self._instruction_response_publisher.publish(response)
        self.reset()

    def gait_rejected(self):
        """Callback called when the state machine rejects the requested gait."""
        response = GaitInstructionResponse(result=GaitInstructionResponse.GAIT_REJECTED)
        self._instruction_response_publisher.publish(response)
        self.reset()

    def gait_finished(self):
        """Callback called when the state machine finishes a gait."""
        response = GaitInstructionResponse(result=GaitInstructionResponse.GAIT_FINISHED)
        self._instruction_response_publisher.publish(response)
        self.reset()

    def _callback_input_device_instruction(self, msg):
        if msg.type == GaitInstruction.STOP:
            self._stopped = True
        elif msg.type == GaitInstruction.GAIT:
            self._gait = msg.gait_name
        elif msg.type == GaitInstruction.PAUSE:
            self._paused = True
        elif msg.type == GaitInstruction.CONTINUE:
            self._paused = False
        elif msg.type == GaitInstruction.INCREMENT_STEP_SIZE:
            self._transition_index = GaitInstruction.INCREMENT_STEP_SIZE
        elif msg.type == GaitInstruction.DECREMENT_STEP_SIZE:
            self._transition_index = GaitInstruction.DECREMENT_STEP_SIZE
        elif msg.type == GaitInstruction.UNKNOWN:
            self._unknown = True
