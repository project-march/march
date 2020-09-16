class GaitStateMachineError(Exception):
    def __init__(self, msg=None):
        base_msg = 'An error occurred in the gait state machine'
        if msg is not None:
            base_msg += ': ' + msg
        super(GaitStateMachineError, self).__init__(msg)
