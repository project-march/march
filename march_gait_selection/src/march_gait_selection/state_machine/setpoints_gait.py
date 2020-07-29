from march_shared_classes.gait.gait import Gait

from .gait_interface import GaitInterface


class SetpointsGait(GaitInterface, Gait):
    def __init__(self, gait_name, subgaits, graph):
        super(SetpointsGait, self).__init__(gait_name, subgaits, graph)
        self._current_subgait = None
        self._should_stop = False
        self._subgait_duration = 0.0

    @property
    def name(self):
        return self.gait_name

    @property
    def starting_position(self):
        subgait = self.subgaits[self.graph.start_subgaits()[0]]
        return dict([(joint.name, joint.setpoints[0].position) for joint in subgait.joints])

    @property
    def final_position(self):
        subgait = self.subgaits[self.graph.end_subgaits()[0]]
        return dict([(joint.name, joint.setpoints[-1].position) for joint in subgait.joints])

    def start(self):
        self._current_subgait = None
        self._should_stop = False
        self._subgait_duration = 0.0

    def update(self, elapsed_time):
        if self._current_subgait is None:
            self._current_subgait = self.graph.start_subgaits()[0]
            self._subgait_duration = 0.0
            return self.subgaits[self._current_subgait].to_joint_trajectory_msg(), False

        self._subgait_duration += elapsed_time
        if self._subgait_duration < self.subgaits[self._current_subgait].duration:
            return None, False
        else:
            if self._should_stop:
                next_subgait = self.graph[(self._current_subgait, self.graph.STOP)]
                self._should_stop = False
            else:
                next_subgait = self.graph[(self._current_subgait, self.graph.TO)]

            if next_subgait == self.graph.END:
                return None, True
            trajectory = self.subgaits[next_subgait].to_joint_trajectory_msg()
            self._current_subgait = next_subgait
            self._subgait_duration = 0.0
            return trajectory, False

    def stop(self):
        transition = self.graph[(self._current_subgait, self.graph.STOP)]
        if transition is not None:
            self._should_stop = True
            return True
        else:
            return False
