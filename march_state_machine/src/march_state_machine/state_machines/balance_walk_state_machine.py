
from .gait_state_machine import GaitStateMachine


class BalanceWalkStateMachine(GaitStateMachine):
    """A smach.StateMachine that implements a balanced walking pattern gait."""

    def __init__(self, gait_name, check_gait_content=False):
        """Initializes the walking pattern gait.

        The gait will be initialized with the following subgaits:
        * right_open
        * right_swing
        * left_swing
        * right_close
        * left_close
        Where right_swing and left_swing can be stopped and will transition
        to either right_close or left_close.

        :type gait_name: str
        :param gait_name: Name of the balanced walking pattern gait
        """
        super(BalanceWalkStateMachine, self).__init__(gait_name, check_gait_content)

        self.open()
        self.add_subgait('right_open_1', succeeded='right_open_2')
        self.add_subgait('right_open_2', succeeded='left_swing_1')

        self.add_subgait('left_swing_1', succeeded='left_swing_2')
        self.add_subgait('left_swing_2', succeeded='right_swing_1', stopped='right_close')

        self.add_subgait('right_swing_1', succeeded='right_swing_2')
        self.add_subgait('right_swing_2', succeeded='left_swing_1', stopped='left_close')

        self.add_subgait('right_close')
        self.add_subgait('left_close')
        self.close()

    def add_outcome(self, label):
        """Add new outcome to the state."""
        self._outcomes.add(label)
