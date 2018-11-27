#!/usr/bin/env python
import rospy
import smach
import smach_ros
from march_api.srv import Trigger

from march_ws.src.march_state_machine.src.launch.ConfigState import ConfigState


def main():
    rospy.init_node("state_machine")

    sm = smach.StateMachine(outcomes=['DONE', 'ERROR'])
    with sm:
        smach.StateMachine.add('CONFIG', ConfigState(), transitions={'succeeded': 'DONE', 'failed': 'ERROR'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
