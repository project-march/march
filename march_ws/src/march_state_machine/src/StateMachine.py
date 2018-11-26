#!/usr/bin/env python
import rospy
import smach
import smach_ros
from march_api.srv import Trigger


class Config(smach.State):
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


def main():
    rospy.init_node("state_machine")

    sm = smach.StateMachine(outcomes=['DONE', 'ERROR'])
    with sm:
        smach.StateMachine.add('CONFIG', Config(), transitions={'succeeded': 'DONE', 'failed': 'ERROR'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
