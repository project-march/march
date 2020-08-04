from march_shared_classes.gait.gait import Gait

from .gait_interface import GaitInterface


class SetpointsGait(GaitInterface, Gait):
    def __init__(self, gait_name, subgaits, graph):
        super(SetpointsGait, self).__init__(gait_name, subgaits, graph)
        self._current_subgait = None
        self._should_stop = False
        self._time_since_start = 0.0

    @property
    def name(self):
        return self.gait_name

    @property
    def subgait_name(self):
        if self._current_subgait in self.subgaits:
            return self.subgaits[self._current_subgait].subgait_name
        else:
            return None

    @property
    def version(self):
        if self._current_subgait in self.subgaits:
            return self.subgaits[self._current_subgait].version
        else:
            return None

    @property
    def duration(self):
        if self._current_subgait in self.subgaits:
            return self.subgaits[self._current_subgait].duration
        else:
            return None

    @property
    def gait_type(self):
        if self._current_subgait in self.subgaits:
            return self.subgaits[self._current_subgait].gait_type
        else:
            return None

    @property
    def starting_position(self):
        subgait = self.subgaits[self.graph.start_subgaits()[0]]
        return dict([(joint.name, joint.setpoints[0].position) for joint in subgait.joints])

    @property
    def final_position(self):
        subgait = self.subgaits[self.graph.end_subgaits()[0]]
        return dict([(joint.name, joint.setpoints[-1].position) for joint in subgait.joints])

    def start(self):
        self._current_subgait = self.graph.start_subgaits()[0]
        self._should_stop = False
        self._time_since_start = 0.0
        return self.subgaits[self._current_subgait].to_joint_trajectory_msg()

    def update(self, elapsed_time):
        self._time_since_start += elapsed_time
        if self._time_since_start < self.subgaits[self._current_subgait].duration:
            return None, False
        else:
            if self._should_stop:
                next_subgait = self.graph[(self._current_subgait, self.graph.STOP)]
                if next_subgait is None:
                    next_subgait = self.graph[(self._current_subgait, self.graph.TO)]
                else:
                    self._should_stop = False
            else:
                next_subgait = self.graph[(self._current_subgait, self.graph.TO)]

            if next_subgait == self.graph.END:
                return None, True
            trajectory = self.subgaits[next_subgait].to_joint_trajectory_msg()
            self._current_subgait = next_subgait
            self._time_since_start = 0.0
            return trajectory, False

    def stop(self):
        if self.graph.is_stoppable():
            self._should_stop = True
            return True
        else:
            return False

    def end(self):
        self._current_subgait = None
