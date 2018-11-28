import rospy
import smach
from march_main.srv import Trigger

import time

class BatteryLow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Battery Low')
        return 'succeeded'
