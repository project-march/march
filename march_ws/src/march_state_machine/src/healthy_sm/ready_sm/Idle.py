import rospy
import smach
from march_main.srv import Trigger

import time

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Idle')
        return 'succeeded'
