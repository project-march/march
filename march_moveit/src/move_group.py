import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from march_shared_resources.msg import Marker

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "right_leg"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

#Get the capture point from the topic. ADD CALLBACK
self._cp_subscriber = rospy.Subscriber('/march/cp_marker_', Marker)

#Homing. CHANGE VALUES
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0   #HAA
joint_goal[1] = 0   #Hip angle
joint_goal[2] = -5  #HFE
joint_goal[3] = 0   #KFE
joint_goal[4] = 2.5 #ADPF
joint_goal[5] = 0   #Ankle angle

