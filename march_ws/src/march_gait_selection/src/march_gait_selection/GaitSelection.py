#!/usr/bin/env python
import os

import rospkg
import rospy
import yaml

from ScheduleGait import ScheduleGait
from march_shared_resources.msg import Subgait


class GaitSelection(object):

    def __init__(self, gait_directory=None, gait_version_map=None, default_yaml=None):
        if gait_directory is None and gait_version_map is None:
            if default_yaml is None:
                rospy.logfatal("Cannot instantiate GaitSelection without parameters, shutting down.")
                raise ValueError('Cannot instantiate GaitSelection without parameters, shutting down.')
            else:
                default_config = yaml.load(open(default_yaml), Loader=yaml.BaseLoader)
                self.gait_directory = os.path.join(
                    rospkg.RosPack().get_path('march_gait_selection'), default_config["directory"])
                self.gait_version_map = default_config["gaits"]

        else:
            self.gait_directory = os.path.join(
                rospkg.RosPack().get_path('march_gait_selection'), gait_directory)
            self.gait_version_map = gait_version_map

    def set_subgait_version(self, gait, subgait, version):
        if self.gait_directory[gait] is None:
            rospy.logerr("Gait " + gait + " does not exist")
        self.gait_directory[gait][subgait] = version

    def get_subgait(self, gait_name, subgait_name):
        subgait = Subgait()
        print self.gait_version_map["walking"]
        subgait_path = self.get_subgait_path(gait_name, subgait_name)

        subgait_yaml = yaml.load(open(subgait_path), Loader=yaml.BaseLoader)

        print(subgait_yaml)
        return subgait_yaml['version']

    def get_subgait_path(self, gait_name, subgait_name):
        print self.gait_directory
        return os.path.join(self.gait_directory, gait_name, subgait_name,
                            self.gait_version_map[gait_name][subgait_name] + '.subgait')


if __name__ == '__main__':
    rospy.init_node("gait_selection")
    server = ScheduleGait()

    default_yaml = os.path.join(rospkg.RosPack().get_path('march_gait_selection'), 'gait', 'default.yaml')
    GaitSelection = GaitSelection(default_yaml)

    rate = rospy.Rate(10)
    rospy.spin()
