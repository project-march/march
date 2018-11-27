import rospy
import smach
from march_api.srv import Trigger


class ConfigState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Checking config')
        checkURDF = rospy.ServiceProxy('march/launch_validation', Trigger)
        result = checkURDF()
        rospy.loginfo(result)
        if result.success:
            return 'succeeded'
        else:
            return 'failed'
