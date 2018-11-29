import rospy
import smach


class BatteryLow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Battery Low')
        return 'succeeded'
