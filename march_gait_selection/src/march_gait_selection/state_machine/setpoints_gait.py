from march_gait_selection.dynamic_gaits.transition_subgait import TransitionSubgait
from march_shared_classes.exceptions.gait_exceptions import GaitError
from march_shared_classes.gait.gait import Gait

from .gait_interface import GaitInterface
from .state_machine_input import TransitionRequest


class SetpointsGait(GaitInterface, Gait):
    def __init__(self, gait_name, subgaits, graph):
        super(SetpointsGait, self).__init__(gait_name, subgaits, graph)
        self._current_subgait = None
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._time_since_start = 0.0

    @property
    def name(self):
        return self.gait_name

    @property
    def subgait_name(self):
        if self._current_subgait is not None:
            return self._current_subgait.subgait_name
        else:
            return None

    @property
    def version(self):
        if self._current_subgait is not None:
            return self._current_subgait.version
        else:
            return None

    @property
    def duration(self):
        if self._current_subgait is not None:
            return self._current_subgait.duration
        else:
            return None

    @property
    def gait_type(self):
        if self._current_subgait is not None:
            return self._current_subgait.gait_type
        else:
            return None

    @property
    def starting_position(self):
        return self.subgaits[self.graph.start_subgaits()[0]].starting_position

    @property
    def final_position(self):
        return self.subgaits[self.graph.end_subgaits()[0]].final_position

    def start(self):
        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._time_since_start = 0.0
        return self._current_subgait.to_joint_trajectory_msg()

    def update(self, elapsed_time):
        self._time_since_start += elapsed_time
        if self._time_since_start < self._current_subgait.duration:
            return None, False

        if self._should_stop:
            next_subgait = self.graph[(self._current_subgait.subgait_name, self.graph.STOP)]
            if next_subgait is None:
                next_subgait = self.graph[(self._current_subgait.subgait_name, self.graph.TO)]
            else:
                self._should_stop = False
        elif self._transition_to_subgait is not None and not self._is_transitioning:
            old_subgait = self.subgaits[self.graph[(self._current_subgait.subgait_name, self.graph.TO)]]
            new_subgait = self.subgaits[self.graph[(self._transition_to_subgait.subgait_name, self.graph.TO)]]
            transition_subgait = TransitionSubgait.from_subgaits(old_subgait, new_subgait, '{s}_transition'.format(
                s=self._transition_to_subgait.subgait_name))
            self._current_subgait = transition_subgait
            self._time_since_start = 0.0
            self._is_transitioning = True
            return transition_subgait.to_joint_trajectory_msg(), False
        elif self._transition_to_subgait is not None and self._is_transitioning:
            next_subgait = self._transition_to_subgait.subgait_name
            self._transition_to_subgait = None
            self._is_transitioning = False
        else:
            next_subgait = self.graph[(self._current_subgait.subgait_name, self.graph.TO)]

        if next_subgait == self.graph.END:
            return None, True
        self._current_subgait = self.subgaits[next_subgait]
        trajectory = self._current_subgait.to_joint_trajectory_msg()
        self._time_since_start = 0.0
        return trajectory, False

    def transition(self, transition_request):
        if self._is_transitioning or self._should_stop:
            return False

        if transition_request == TransitionRequest.DECREASE_SIZE:
            name = self.graph[(self._current_subgait.subgait_name, self.graph.DECREASE_SIZE)]
        elif transition_request == TransitionRequest.INCREASE_SIZE:
            name = self.graph[(self._current_subgait.subgait_name, self.graph.INCREASE_SIZE)]
        else:
            return False

        if name is not None:
            self._transition_to_subgait = self.subgaits[name]
            return True
        return False

    def stop(self):
        if self.graph.is_stoppable() and not self._is_transitioning and self._transition_to_subgait is None:
            self._should_stop = True
            return True
        else:
            return False

    def end(self):
        self._current_subgait = None

    def set_subgait_versions(self, robot, gait_directory, version_map):
        if self._current_subgait is None:
            super(SetpointsGait, self).set_subgait_versions(robot, gait_directory, version_map)
        else:
            raise GaitError('Cannot change subgait version while gait is being executed')
