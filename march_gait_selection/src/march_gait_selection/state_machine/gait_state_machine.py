class GaitStateMachine(object):
    def __init__(self, gait_selection):
        self._gait_selection = gait_selection
        self._graph = self._generate_graph()

    def _generate_graph(self):
        graph = {}
        idle_states = {}
        for gait in self._gait_selection:
            final_position = gait.final_position()
            if final_position in idle_states:
                idle_states[final_position].append(gait.name())
            else:
                idle_states[final_position] = [gait.name()]

        position_map = dict([(gait.final_position(), gait.name()) for gait in self._gait_selection])
        return graph
