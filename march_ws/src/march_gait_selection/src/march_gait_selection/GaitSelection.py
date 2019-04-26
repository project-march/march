#!/usr/bin/env python
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal, \
    FollowJointTrajectoryResult
from std_msgs.msg import String
from march_shared_resources.msg import MoveToPoseAction, MoveToPoseGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class TargetPoseAction(object):
    _action_server = None
    _trajectory_publisher = None
    _trajectory_client = None

    def callback(self, goal):
        rospy.loginfo(rospy.get_caller_id() + "I heard pose: %s", goal.pose)
        result = self.joint_trajectory_client(goal.pose)
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.loginfo("set_succeeded")
            self._action_server.set_succeeded()
        else:
            rospy.loginfo("set_aborted")
            self._action_server.set_aborted()

    def joint_trajectory_client(self, pose_name):
        trajectory_message = FollowJointTrajectoryActionGoal()
        trajectory_message.goal.trajectory.joint_names = ["left_hip", "left_knee", "left_ankle", "right_hip",
                                                          "right_knee", "right_ankle"]
        if pose_name == "sit":
            point = JointTrajectoryPoint()
            point.positions = [1.3, 1.3, 0.349065850399, 1.3, 1.3, 0.349065850399]
            point.velocities = [0, 0, 0, 0, 0, 0]
            point.time_from_start = rospy.Duration.from_sec(3)
        else:
            point = JointTrajectoryPoint()
            point.positions = [0, 0, 0, 0, 0, 0]
            point.velocities = [0, 0, 0, 0, 0, 0]
            point.time_from_start = rospy.Duration.from_sec(3)
        trajectory_message.goal.trajectory.points.append(point)
        rospy.loginfo("send_goal:")
        self._trajectory_client.send_goal(trajectory_message.goal)
        rospy.loginfo("wait_for_result:")
        self._trajectory_client.wait_for_result()
        rospy.loginfo("return result:")
        return self._trajectory_client.get_result()

    def __init__(self):
        rospy.loginfo("starting gait_selection")
        self._action_server = actionlib.SimpleActionServer("march/target_pose", MoveToPoseAction,
                                                           execute_cb=self.callback, auto_start=False)
        self._action_server.start()
        self._trajectory_client = actionlib.SimpleActionClient("exo_controller/follow_joint_trajectory",
                                                               FollowJointTrajectoryAction)
        self._trajectory_client.wait_for_server()


if __name__ == '__main__':
    rospy.init_node("gait_selection")
    server = TargetPoseAction()
    _trajectory_publisher = rospy.Publisher('exo_controller/follow_joint_trajectory', String, queue_size=10)
    rate = rospy.Rate(10)
    rospy.spin()
