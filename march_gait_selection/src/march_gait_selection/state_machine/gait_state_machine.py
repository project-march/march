import rospy

from .gait_state_machine_error import GaitStateMachineError
from .home_gait import HomeGait


class GaitStateMachine(object):
    UNKNOWN = 'unknown'

    def __init__(self, gait_selection):
        self._gait_selection = gait_selection
        self._home_gaits = {}
        self._idle_transitions = {}
        self._gait_transitions = {}
        self._generate_graph()

        self._current_state = self.UNKNOWN
        self._is_idle = True

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
            if gait_name in self._gait_transitions:
                self._gait_transitions[gait_name].add(idle_name)
            else:
                self._gait_transitions[gait_name] = {idle_name}

        self._validate_transitions()

        self._generate_home_gaits(idle_positions)

    def _validate_transitions(self):
        for name, idles in self._gait_transitions.items():
            for idle in idles:
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
