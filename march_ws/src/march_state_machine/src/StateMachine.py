#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
import time
from random import randint

from smach import State
from std_msgs.msg import Empty


class start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_done'])

    def execute(self, userdata):
        return 'start_done'


class Config(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        time.sleep(1)
        rospy.loginfo('Checking config')
        return 'succeeded'


# define state Launch
class Hardware(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        time.sleep(1)
        rospy.loginfo('Checking hardware')
        return 'succeeded'


# define state Launch
class Errorlistener(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed'])

    def execute(self, userdata):
        rospy.loginfo('Listening for errors...')
        return 'failed'


# gets called when ANY child state terminates
def child_term_cb(outcome_map):
    # terminate all running states if FOO finished with outcome 'outcome3'
    if outcome_map['ERRORLISTENER'] == 'failed':
        print("Terminating all children")
        return True

    # in all other case, just keep running, don't terminate anything
    return False


def main():
    rospy.init_node("preemption_example")

    launch_concurrence = smach.Concurrence(outcomes=['succeeded', 'aborted'],
                                           default_outcome='aborted',
                                           child_termination_cb=child_term_cb,
                                           outcome_map={
                                               'aborted': {'ERRORLISTENER': 'failed'},
                                               'succeeded': {'LAUNCH': 'succeeded'}}
                                           )

    with launch_concurrence:
        launch_sequence = smach.StateMachine(outcomes = ['preempted', 'succeeded'])
        with launch_sequence:
            smach.StateMachine.add('CONFIG', Config(), transitions={'succeeded': 'HARDWARE'})
            smach.StateMachine.add('HARDWARE', Hardware())

        smach.Concurrence.add('LAUNCH', launch_sequence)
        smach.Concurrence.add('ERRORLISTENER', Errorlistener())

    sm = smach.StateMachine(outcomes=['DONE', 'ERROR'])
    with sm:
        smach.StateMachine.add('SETUP', start(), transitions={'start_done': 'LAUNCH_CONCUR'})
        smach.StateMachine.add('LAUNCH_CONCUR', launch_concurrence,
                               transitions={'succeeded': 'DONE', 'aborted': 'ERROR'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
