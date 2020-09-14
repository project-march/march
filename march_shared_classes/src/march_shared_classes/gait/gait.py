import os

import yaml

from march_shared_classes.exceptions.gait_exceptions import GaitNameNotFound, NonValidGaitContent, SubgaitNameNotFound
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError

from .subgait import Subgait
from .subgait_graph import SubgaitGraph


class Gait(object):
    """base class for a generated gait."""

    def __init__(self, gait_name, subgaits, graph):
        """Initializes and verifies the gait.

        :param str gait_name: Name of the gait
        :param dict subgaits: Mapping of names to subgait instances
        :param SubgaitGraph graph: Mapping of subgait names transitions
        """
        self.gait_name = gait_name
        self.subgaits = subgaits
        self.graph = graph

        self._validate_trajectory_transition()

    @classmethod
    def from_file(cls, gait_name, gait_directory, robot, gait_version_map):
        """Extract the data from the .gait file.

        :param gait_name:
            name of the gait to unpack
        :param gait_directory:
            path of the directory where the .gait file is located
        :param robot:
            the robot corresponding to the given .gait file
        :param gait_version_map:
            The parsed yaml file which states the version of the subgaits
        """
        gait_folder = gait_name
        gait_path = os.path.join(gait_directory, gait_folder, gait_name + '.gait')
        if not os.path.isfile(gait_path):
            raise FileNotFoundError(gait_path)

        with open(gait_path, 'r') as gait_file:
            gait_dictionary = yaml.load(gait_file, Loader=yaml.SafeLoader)

        return cls.from_dict(robot, gait_dictionary, gait_directory, gait_version_map)

    @classmethod
    def from_dict(cls, robot, gait_dictionary, gait_directory, gait_version_map):
        """Create a new gait object using the .gait and .subgait files.

        :param robot:
            the robot corresponding to the given .gait file
        :param gait_dictionary:
            the information of the .gait file as a dictionary
        :param gait_directory:
            path of the directory where the .gait file is located
        :param gait_version_map:
            The parsed yaml file which states the version of the subgaits

        :return:
            If the data in the files is validated a gait object is returned
        """
        gait_name = gait_dictionary['name']
        subgaits = gait_dictionary['subgaits']

        graph = SubgaitGraph(subgaits)
        subgaits = dict([(name, cls.load_subgait(robot, gait_directory, gait_name, name, gait_version_map))
                         for name in subgaits if name not in ('start', 'end')])

        return cls(gait_name, subgaits, graph)

    @staticmethod
    def load_subgait(robot, gait_directory, gait_name, subgait_name, gait_version_map):
        """Read the .subgait file and extract the data.

        :returns
            if gait and subgait names are valid return populated Gait object
        """
        if gait_name not in gait_version_map:
            raise GaitNameNotFound(gait_name)
        if subgait_name not in gait_version_map[gait_name]:
            raise SubgaitNameNotFound(subgait_name, gait_name)

        version = gait_version_map[gait_name][subgait_name]
        return Subgait.from_name_and_version(robot, gait_directory, gait_name, subgait_name, version)

    def _validate_trajectory_transition(self):
        """Compares and validates the trajectory end and start points."""
        for from_subgait_name, to_subgait_name in self.graph:
            if len({from_subgait_name, to_subgait_name} & {self.graph.START, self.graph.END}) > 0:
                continue

            from_subgait = self.subgaits[from_subgait_name]
            to_subgait = self.subgaits[to_subgait_name]

            if not from_subgait.validate_subgait_transition(to_subgait):
                raise NonValidGaitContent(msg='Gait {gait} with end setpoint of subgait {sn} to subgait {ns} '
                                              'does not match'.format(gait=self.gait_name,
                                                                      sn=from_subgait.subgait_name,
                                                                      ns=to_subgait.subgait_name))

    def set_subgait_versions(self, robot, gait_directory, version_map):
        """Updates the given subgait versions and verifies transitions.

        :param robot: URDF matching subgaits
        :param str gait_directory: path to the gait directory
        :param dict version_map: Mapping subgait names to versions
        """
        new_subgaits = {}
        for subgait_name, version in version_map.items():
            if subgait_name not in self.subgaits:
                raise SubgaitNameNotFound(subgait_name, self.gait_name)
            new_subgaits[subgait_name] = Subgait.from_name_and_version(robot, gait_directory, self.gait_name,
                                                                       subgait_name, version)

        for from_subgait_name, to_subgait_name in self.graph:
            if from_subgait_name in new_subgaits or to_subgait_name in new_subgaits:
                if from_subgait_name == self.graph.START:
                    old_subgait = self.subgaits[to_subgait_name]
                    new_subgait = new_subgaits[to_subgait_name]

                    if old_subgait.starting_position != new_subgait.starting_position:
                        raise NonValidGaitContent(
                            msg='The starting position of new version {gait} {subgait} does not match'.format(
                                gait=self.gait_name, subgait=to_subgait_name))
                elif to_subgait_name == self.graph.END:
                    old_subgait = self.subgaits[from_subgait_name]
                    new_subgait = new_subgaits[from_subgait_name]

                    if old_subgait.final_position != new_subgait.final_position:
                        raise NonValidGaitContent(
                            msg='The final position of new version {gait} {subgait} does not match'.format(
                                gait=self.gait_name, subgait=from_subgait_name))
                else:
                    from_subgait = new_subgaits.get(from_subgait_name, self.subgaits[from_subgait_name])
                    to_subgait = new_subgaits.get(to_subgait_name, self.subgaits[to_subgait_name])

                    if not from_subgait.validate_subgait_transition(to_subgait):
                        raise NonValidGaitContent(
                            msg='Gait {gait} with end setpoint of subgait {sn} to subgait {ns} does not match'.format(
                                gait=self.gait_name, sn=from_subgait.subgait_name, ns=to_subgait.subgait_name))

        self.subgaits.update(new_subgaits)

    def __getitem__(self, name):
        """Returns a subgait from the loaded subgaits."""
        return self.subgaits.get(name)
