#!/usr/bin/env python
import os

import yaml
import rospkg
import rospy

from march_shared_resources.msg import Subgait
from trajectory_msgs.msg import JointTrajectory
from rospy_message_converter import message_converter


class GaitSelection(object):

    def __init__(self, gait_directory=None, gait_version_map=None, default_yaml=None):
        if gait_directory is None and gait_version_map is None:
            if default_yaml is None:
                rospy.logfatal("Cannot instantiate GaitSelection without parameters, shutting down.")
                raise ValueError('Cannot instantiate GaitSelection without parameters, shutting down.')
            else:
                default_config = yaml.load(open(default_yaml), Loader=yaml.SafeLoader)
                self.gait_directory = os.path.join(
                    rospkg.RosPack().get_path('march_gait_selection'), default_config["directory"])
                self.gait_version_map = default_config["gaits"]
        else:
            self.gait_directory = os.path.join(
                rospkg.RosPack().get_path('march_gait_selection'), gait_directory)
            self.gait_version_map = gait_version_map
        rospy.loginfo("GaitSelection initialized with gait_directory " + self.gait_directory)
        rospy.loginfo("GaitSelection initialized with gait_version_map " + str(self.gait_version_map))

    def set_subgait_version(self, gait_name, subgait_name, version):
        if self.validate_subgait_name(gait_name, subgait_name):
            if self.validate_version_name(gait_name, subgait_name, version):
                self.gait_version_map[gait_name][subgait_name] = version
                return True
        return False

    def get_subgait(self, gait_name, subgait_name):
        try:
            subgait_path = self.get_subgait_path(gait_name, subgait_name)
        except KeyError:
            rospy.logerr("Could not find subgait " + gait_name + "/" + subgait_name)
            return Subgait()

        subgait_yaml = yaml.load(open(subgait_path), Loader=yaml.SafeLoader)
        subgait = message_converter.convert_dictionary_to_ros_message('march_shared_resources/Subgait', subgait_yaml)
        return subgait

    def validate_subgait_name(self, gait_name, subgait_name):
        try:
            self.gait_version_map[gait_name]
        except KeyError:
            rospy.logerr("Gait " + gait_name + " does not exist")
            return False
        try:
            self.gait_version_map[gait_name][subgait_name]
        except KeyError:
            rospy.logerr("Subgait " + subgait_name + " does not exist")
            return False
        return True

    def validate_version_name(self, gait_name, subgait_name, version):
        if len(gait_name) == 0 or len(subgait_name) == 0 or len(version) == 0:
            return False

        subgait_path = os.path.join(self.gait_directory, gait_name, subgait_name, version + '.subgait')
        try:
            open(subgait_path)
        except IOError:
            return False
        return True

    def get_subgait_path(self, gait_name, subgait_name):
        if not self.validate_subgait_name(gait_name, subgait_name):
            raise KeyError(
                "Could not find subgait " + gait_name + "/" + subgait_name +
                " in the mapping" + str(self.gait_version_map))
        if not self.validate_version_name(gait_name, subgait_name, self.gait_version_map[gait_name][subgait_name]):
            raise KeyError(
                "Could not find subgait file " + gait_name + "/" + subgait_name + "/" +
                self.gait_version_map[gait_name][subgait_name] + ".subgait")

        return os.path.join(self.gait_directory, gait_name, subgait_name,
                            self.gait_version_map[gait_name][subgait_name] + '.subgait')

    def scan_directory(self):
        """Scan the gait_directory recursively and create a dictionary of all subgait files"""
        rootdir = self.gait_directory.rstrip(os.sep)

        gait_dict = {}
        for gait in os.listdir(rootdir):
            gait_path = os.path.join(rootdir, gait)
            if not os.path.isdir(gait_path):
                continue

            subgait_dict = {}
            for subgait in os.listdir(gait_path):
                subgait_path = os.path.join(gait_path, subgait)
                if not os.path.isdir(subgait_path):
                    continue
                versions = []
                for version in os.listdir(os.path.join(subgait_path)):
                    versions.append(version)
                subgait_dict[subgait] = versions
            gait_dict[gait] = subgait_dict
        return gait_dict

    def validate_version_map(self, map):
        # Check if all subgaits in the map exist.
        for gait in map:
            for subgait in map[gait]:
                version = map[gait][subgait]
                if not self.validate_version_name(gait, subgait, version):
                    return False
        return True

    def validate_gait(self, gait):
        if len(gait.from_subgait) != len(gait.to_subgait):
            return False

        for i in range(0, len(gait.from_subgait)):
            pass


    def validate_trajectory_transition(self, old_trajectory, new_trajectory):
        return old_trajectory.names == new_trajectory.names and old_trajectory.points[-1] == new_trajectory.points[0]