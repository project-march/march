#!/usr/bin/env python
import rospy
import smach
import smach_ros
from march_api.srv import Trigger

import launch_sm.launch_sm as launch_sm


class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        return 'succeeded'


def main():
    rospy.init_node("state_machine")

    sm = smach.StateMachine(outcomes=['READY', 'SHUTDOWN'])
    with sm:
        smach.StateMachine.add('START', Start(), transitions={'succeeded': 'LAUNCH'})

        smach.StateMachine.add('LAUNCH', launch_sm.create(),
                               transitions={'succeeded': 'READY', 'failed': 'SHUTDOWN'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
