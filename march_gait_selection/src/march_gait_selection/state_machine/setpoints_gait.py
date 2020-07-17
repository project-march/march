from march_shared_classes.gait.gait import Gait

from .gait_interface import GaitInterface


class SetpointsGait(GaitInterface, Gait):
    def starting_position(self):
        subgait = self.subgaits[self.graph.start_subgaits()[0]]
        return dict([(joint.name, joint.setpoints[0].position) for joint in subgait.joints])

    def final_position(self):
        subgait = self.subgaits[self.graph.end_subgaits()[0]]
        return dict([(joint.name, joint.setpoints[-1].position) for joint in subgait.joints])

    def name(self):
        return self.gait_name
