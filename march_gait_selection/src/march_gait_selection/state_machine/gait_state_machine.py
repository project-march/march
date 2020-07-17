import rospy


class GaitStateMachine(object):
    def __init__(self, gait_selection):
        self._gait_selection = gait_selection
        self._graph = self._generate_graph()

    def _generate_graph(self):
        idle_transitions = {}
        idle_positions = {}
        gait_transitions = {}
        for gait in self._gait_selection:
            gait_name = gait.name()
            starting_position = gait.starting_position()
            idle_name = next((name for name, position in idle_positions.items() if position == starting_position), None)
            if idle_name is None:
                idle_name = 'idle_{0}'.format(len(idle_positions))
                idle_positions[idle_name] = starting_position
            if idle_name in idle_transitions:
                idle_transitions[idle_name].add(gait_name)
            else:
                idle_transitions[idle_name] = {gait_name}

            final_position = gait.final_position()
            idle_name = next((name for name, position in idle_positions.items() if position == final_position), None)
            if idle_name is None:
                idle_name = 'idle_{0}'.format(len(idle_positions))
                idle_positions[idle_name] = final_position
            if gait_name in gait_transitions:
                gait_transitions[gait_name].add(idle_name)
            else:
                gait_transitions[gait_name] = {idle_name}
        for name, gaits in idle_transitions.items():
            for gait in gaits:
                if gait not in gait_transitions:
                    rospy.logwarn('{0} does not have transitions'.format(gait))
        for name, idles in gait_transitions.items():
            for idle in idles:
                if idle not in idle_transitions:
                    rospy.logwarn('{0} does not have transitions'.format(idle))

        return idle_transitions, gait_transitions
