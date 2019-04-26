#!/usr/bin/env python
import rospy
import actionlib

from std_msgs.msg import String
from march_shared_resources.msg import MoveToPoseAction, MoveToPoseGoal


class TargetPoseAction(object):
    _action_server = None

    def callback(self, goal):
        rospy.loginfo(rospy.get_caller_id() + "I heard pose: %s", goal.pose)
        self._action_server.set_succeeded()

    def __init__(self):
        rospy.loginfo("starting gait_selection")
        self._action_server = actionlib.SimpleActionServer("march/target_pose", MoveToPoseAction,
                                                           execute_cb=self.callback, auto_start=False)
        self._action_server.start()


if __name__ == '__main__':
    rospy.init_node("gait_selection")
    server = TargetPoseAction()
    pub = rospy.Publisher('exo_controller/follow_joint_trajectory', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()
    rospy.spin()
