import rospy
import smach
from march_api.srv import Trigger


class UrdfState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Checking config')
        checkUrdf = rospy.ServiceProxy('march/urdf_validation', Trigger)
        result = checkUrdf()
        rospy.loginfo(result)
        if result.success:
            return 'succeeded'
        else:
            return 'failed'
