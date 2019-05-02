import rospy
import actionlib

from march_shared_resources.msg import GaitNameAction, GaitAction, GaitNameActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class ScheduleGait(object):
    perform_gait_server = None
    schedule_gait_client = None

    def __init__(self):
        self.perform_gait_server = actionlib.SimpleActionServer("march/gait/perform", GaitNameAction,
                                                                execute_cb=self.target_gait_callback,
                                                                auto_start=False)
        self.perform_gait_server.start()

        self.schedule_gait_client = actionlib.SimpleActionClient("march/gait/schedule",
                                                                 GaitAction)
        self.schedule_gait_client.wait_for_server()

    def target_gait_callback(self, goal):
        rospy.loginfo(" %s pose requested", goal.subgait_name)
        trajectory_result = self.schedule_gait(goal.subgait_name)

    def schedule_gait(self, gait_name):
        trajectory_message = GaitNameActionGoal()
        trajectory_message.goal.trajectory.joint_names = ["left_hip", "left_knee", "left_ankle", "right_hip",
                                                          "right_knee", "right_ankle"]
        # For now we handle everything with 2 gaits sit and stand.
        # @TODO(Isha) implement proper gait selection
        if gait_name == "sit":
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
        self.schedule_gait_client.send_goal(trajectory_message.goal)
        rospy.logdebug("wait_for_result:")
        self.schedule_gait_client.wait_for_result()
        return self.schedule_gait_client.get_result()
