import rospy
import smach
from march_main.srv import Trigger

import time

class ConfigState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Checking config')
        checkConfig = rospy.ServiceProxy('march/config_validation', Trigger)
        result = checkConfig()
        self.request_preempt()
        rospy.loginfo(result)
        if result.success:
            return 'succeeded'
        else:
            return 'failed'

    def service_preempt(self):
        rospy.logwarn("SERVICE PREEMPT")
        return 'failed'
