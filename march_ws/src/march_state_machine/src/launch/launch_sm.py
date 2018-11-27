#!/usr/bin/env python
import rospy
import smach
import smach_ros
from march_api.srv import Trigger

from march_ws.src.march_state_machine.src.launch.ConfigState import ConfigState


sm_launch = smach.StateMachine(outcomes=['succeeded', 'failed'])
# Open the container
with sm_launch:

    # Add states to the container
    smach.StateMachine.add('CONFIG', ConfigState(),
                           transitions={'succeeded':'URDF',
                                        'failed':'SHUTDOWN'})
    smach.StateMachine.add('URDF', urdfState(),
                           transitions={'succeeded':'URDF',
                                        'failed':'SHUTDOWN'})

smach.StateMachine.add('LAUNCH', sm_launch,
                       transitions={'succeeded':'READY', 'failed': 'SHUTDOWN'})

if __name__ == "__main__":
    main()
