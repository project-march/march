import rospy
import smach
from march_main.srv import Trigger

import time

class UrdfState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Checking config')
        time.sleep(5)
        checkUrdf = rospy.ServiceProxy('march/urdf_validation', Trigger)

        result = checkUrdf()
        rospy.loginfo(result)
        if result.success:
            return 'succeeded'
        else:
            return 'failed'
