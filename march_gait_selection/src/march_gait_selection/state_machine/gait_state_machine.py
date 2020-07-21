import rospy

from march_gait_selection.gait_selection import GaitSelection

from .gait_state_machine_error import GaitStateMachineError
from .home_gait import HomeGait
from .state_machine_input import StateMachineInput


class GaitStateMachine(object):
    UNKNOWN = 'unknown'

    def __init__(self, gait_selection, state_input):
        """Generates a state machine from given gaits and resets it to UNKNOWN state.

        In order to start the state machine see `run`.

        :param GaitSelection gait_selection: Selection of loaded gaits to build from
        :param StateMachineInput state_input: Input interface for controlling the states
        """
        self._gait_selection = gait_selection
        self._input = state_input
        self._home_gaits = {}
        self._idle_transitions = {}
        self._gait_transitions = {}
        self._generate_graph()

        self._current_state = self.UNKNOWN
        self._current_gait = None
        self._is_idle = True
        self._shutdown_requested = False

    def run(self):
        """Runs the state machine until shutdown is requested."""
        rate = rospy.Rate(30.0)
        last_update_time = rospy.Time.now()
        while not self._shutdown_requested:
            now = rospy.Time.now()
            elapsed_time = now - last_update_time
            last_update_time = now
            if self._is_idle:
                self._process_idle_state()
            else:
                self._process_gait_state(elapsed_time.to_sec())
            rate.sleep()

    def request_shutdown(self):
        """Requests shutdown, which will terminate the state machine as soon as possible."""
        self._shutdown_requested = True

    def _process_idle_state(self):
        if self._input.gait_requested():
            gait_name = self._input.gait_name()
            rospy.loginfo('Requested gait ' + gait_name)
            if gait_name in self._idle_transitions[self._current_state]:
                self._current_state = gait_name
                self._is_idle = False
                self._input.gait_accepted()
                rospy.loginfo('Accepted')
            else:
                self._input.gait_rejected()
                rospy.loginfo('Rejected')

    def _process_gait_state(self, elapsed_time):
        if self._current_gait is None:
            if self._current_state in self._home_gaits:
                self._current_gait = self._home_gaits[self._current_state]
            else:
                self._current_gait = self._gait_selection[self._current_state]
            self._current_gait.start()
            rospy.loginfo('Executing ' + self._current_gait.name())

        if self._input.stop_requested():
            self._current_gait.stop()

        trajectory, should_stop = self._current_gait.update(elapsed_time)
        # schedule trajectory if any

        if should_stop:
            self._current_state = self._gait_transitions[self._current_state]
            self._is_idle = True
            self._current_gait.end()
            self._current_gait = None
            self._input.gait_finished()

    def _generate_graph(self):
        self._idle_transitions = {}
        self._gait_transitions = {}
        idle_positions = self._gait_selection.positions
        for gait in self._gait_selection:
            gait_name = gait.name()
            starting_position = gait.starting_position()
            idle_name = next((name for name, position in idle_positions.items() if position == starting_position), None)
            if idle_name is None:
                idle_name = 'unknown_idle_{0}'.format(len(idle_positions))
                rospy.logwarn('No named position given for starting position of gait `{gn}`, creating `{n}`'
                              .format(gn=gait_name, n=idle_name))
                idle_positions[idle_name] = starting_position
            if idle_name in self._idle_transitions:
                self._idle_transitions[idle_name].add(gait_name)
            else:
                self._idle_transitions[idle_name] = {gait_name}

            final_position = gait.final_position()
            idle_name = next((name for name, position in idle_positions.items() if position == final_position), None)
            if idle_name is None:
                idle_name = 'unknown_idle_{0}'.format(len(idle_positions))
                rospy.logwarn('No named position given for final position of gait `{gn}`, creating `{n}`'
                              .format(gn=gait_name, n=idle_name))
                idle_positions[idle_name] = final_position
            self._gait_transitions[gait_name] = idle_name

        self._validate_transitions()

        self._generate_home_gaits(idle_positions)

    def _validate_transitions(self):
        for name, idle in self._gait_transitions.items():
            if idle not in self._idle_transitions:
                rospy.logwarn('{0} does not have transitions'.format(idle))

    def _generate_home_gaits(self, idle_positions):
        self._idle_transitions[self.UNKNOWN] = set()
        home_gaits = {}
        for idle_name, position in idle_positions.items():
            home_gait = HomeGait(idle_name, position)
            home_gait_name = home_gait.name()
            home_gaits[home_gait_name] = home_gait
            if home_gait_name in self._gait_transitions:
                raise GaitStateMachineError('Gaits cannot have the same name as home gait `{0}`'.format(home_gait_name))
            self._gait_transitions[home_gait_name] = idle_name
            self._idle_transitions[self.UNKNOWN].add(home_gait_name)
