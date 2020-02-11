#!/usr/bin/env python
from march_moveit import move_group
# import sys
# import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
#
# from math import pi,asin,sqrt
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
# from march_shared_resources.msg import Marker

if __name__ == '__main__':
    move_group.main()
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_group', anonymous=True)
    #
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    #
    # rospy.wait(10)
    #
    # group_name = "right_leg"
    # move_group_right = moveit_commander.MoveGroupCommander(group_name)
    # group_name = "left_leg"
    # move_group_left = moveit_commander.MoveGroupCommander(group_name)
    #
    # #Homing
    # # joint_goal_right = move_group_right.get_current_joint_values() DOESNT WORK BECAUSE WE PUBLISH AT MARCH/JOINT_STATES
    # joint_goal_right = []
    # joint_goal_right.append(0) #RHAA
    # joint_goal_right.append(-5*pi/180) #RHFE
    # joint_goal_right.append(0) #RKFE
    # joint_goal_right.append(2.5*pi/180) #RADPF
    # joint_goal_right.append(0) #LHAA
    # joint_goal_right.append(0) #LHFE
    # joint_goal_right.append(0) #LKFE
    # joint_goal_right.append(0) #LADPF
    #
    # #joint_goal_left = move_group_left.get_current_joint_values() DOESNT WORK BECAUSE WE PUBLISH AT MARCH/JOINT_STATES
    # joint_goal_left = []
    # joint_goal_left.append(0) #LHAA
    # joint_goal_left.append(-5*pi/180) #LHFE
    # joint_goal_left.append(0) #LKFE
    # joint_goal_left.append(2.5*pi/180) #LADPF
    # joint_goal_left.append(0) #RHAA
    # joint_goal_left.append(-5*pi/180) #RHFE
    # joint_goal_left.append(0) #RKFE
    # joint_goal_left.append(2.5*pi/180) #RADPF
    #
    # move_group_right.go(joint_goal_right, wait=True)
    # move_group_left.go(joint_goal_left, wait=True)
    # move_group_left.stop()
    # move_group_right.stop()
    #
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                                moveit_msgs.msg.DisplayTrajectory,
    #                                                queue_size=20)
    #
    # #Get the capture point from the topic.
    # cp_subscriber = rospy.Subscriber('/march/cp_marker_ankle_plate_right', Marker,capture_point_callback_right)
    # cp_subscriber = rospy.Subscriber('/march/cp_marker_ankle_plate_left', Marker,capture_point_callback_left)
    #
    # def capture_point_callback_right(data):
    #     pose_goal = data.pose
    #     move_group_right.set_pose_target(pose_goal)
    #     plan = move_group_right.go(wait=True)
    #
    # def capture_point_callback_left(data):
    #     pose_goal = data.pose
    #     move_group_left.set_pose_target(pose_goal)
    #     plan = move_group_left.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    # move_group_right.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    # move_group_right.clear_pose_targets()

    # def capture_point_callback(data):
#     y =
#     y1 =
#     y2 =
#     L =
#     L1 =
#     L2 =
#     LL = sqrt(L**2 - y**2 + 2*y*y2)
#     joint_goal_right_cp.append(0)
#     joint_goal_right_cp.append(asin(y1/L))
