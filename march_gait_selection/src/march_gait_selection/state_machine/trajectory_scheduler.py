import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
import rospy


class TrajectoryScheduler(object):
    def __init__(self, topic):
        self._failed = False
        self._trajectory_client = actionlib.SimpleActionClient(topic, FollowJointTrajectoryAction)

    def schedule(self, trajectory):
        """Schedules a new trajectory.

        :param JointTrajectory trajectory: a trajectory for all joints to follow
        """
        self._failed = False
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._trajectory_client.send_goal(goal, done_cb=self._done_cb)

    def failed(self):
        return self._failed

    def reset(self):
        self._failed = False

    def _done_cb(self, _state, result):
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.logerr('Failed to execute trajectory. {0} ({1})'.format(result.error_string, result.error_code))
            self._failed = True
