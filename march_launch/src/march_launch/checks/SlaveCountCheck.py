import os
import subprocess

import rospy
from march_launch.Color import Color
from SoftwareCheck import SoftwareCheck
from urdf_parser_py import urdf


class SlaveCountCheck(SoftwareCheck):

    def __init__(self):
        SoftwareCheck.__init__(self, "SlaveCount", "", 10, True)

    def perform(self):
        slave_count = os.system('ssh march@march rosrun march_hardware slave_count_check')
        # slave_count = os.system('rosrun march_hardware slave_count_check')
        self.log("Ethercat found " + str(slave_count) + " slave(s)", Color.Info)
        self.done = True
        self.passed = slave_count > 0
