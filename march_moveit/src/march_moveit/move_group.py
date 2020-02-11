import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros

from math import pi,asin,sqrt, acos

class MoveGroup(object):

    def __init__(self):

        rospy.init_node('move_group', anonymous=True)\

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "right_leg"
        self.move_group_right = moveit_commander.MoveGroupCommander(group_name)
        group_name = "left_leg"
        self.move_group_left = moveit_commander.MoveGroupCommander(group_name)

        self._cp_subscriber_right = rospy.Subscriber('/march/cp_marker_ankle_plate_right', Marker, self.capture_point_callback_right)
        self._cp_subscriber_left = rospy.Subscriber('/march/cp_marker_ankle_plate_left', Marker, self.capture_point_callback_left)

        self.capture_point_right = Pose()
        self.capture_point_left = Pose()

        joint_goal_start = []
        joint_goal_start.append(0) #LHAA
        joint_goal_start.append(-5*pi/180) #LHFE
        joint_goal_start.append(0) #LKFE
        joint_goal_start.append(2.5*pi/180) #LADPF
        joint_goal_start.append(0) #RHAA
        joint_goal_start.append(-5*pi/180) #RHFE
        joint_goal_start.append(0) #RKFE
        joint_goal_start.append(2.5*pi/180) #RADPF

        self.move_group_right.go(joint_goal_start, wait=True)
        self.left_swing()
        self.right_swing()

    def capture_point_callback_right(self, data):
        self.capture_point_right = data.pose

    def capture_point_callback_left(self, data):
        self.capture_point_left = data.pose

    def right_swing(self):
        trans = self.listener.lookupTransform("world", "ankle_plate_right", rospy.Time(0))
        y = self.capture_point_right.position.y - trans.transform.translation.y
        # rospy.loginfo(self.tf_buffer.canTransform("world", "ankle_plate_right",rospy.Time()))
        y1 = 0.6 * y
        y2 = y - y1
        Lu = 0.385
        Ll = 0.385
        L = Lu + Ll
        LL = sqrt(L**2 -y**2 + 2*y*y2)
        beta = acos((Ll**2 - LL**2 - Lu**2) / (-2*LL*Lu))
        joint_goal = []
        joint_goal.append(0) #LHAA
        joint_goal.append(asin(y1/L)) #LHFE
        joint_goal.append(0) #LKFE
        joint_goal.append(asin(y1/L)) #LADPF
        joint_goal.append(0) #RHAA
        joint_goal.append(asin(y2/LL)+beta) #RHFE
        joint_goal.append(acos((2*Ll*Lu-y**2+2*y*y2)/(-2*Ll*Lu))) #RKFE
        joint_goal.append(-asin(y2/L)) #RADPF
        rospy.loginfo(joint_goal)
        plan = self.move_group_right.go(joint_goal, wait=True)
        return plan

    def left_swing(self):
        # trans = self.listener.lookupTransform("world", "ankle_plate_right", rospy.Time(0))
        trans = self.tf_buffer.lookup_transform("world", "ankle_plate_left", rospy.Time.now(), rospy.Duration(1.0))
        y = self.capture_point_right.position.y - trans.transform.translation.y
        # rospy.loginfo(self.tf_buffer.canTransform("world", "ankle_plate_right",rospy.Time()))
        y1 = 0.6 * y
        y2 = y - y1
        Lu = 0.385
        Ll = 0.385
        L = Lu + Ll
        LL = sqrt(L**2 -y**2 + 2*y*y2)
        beta = acos((Ll**2 - LL**2 - Lu**2) / (-2*LL*Lu))
        joint_goal = []
        joint_goal.append(0) #LHAA
        joint_goal.append(asin(y2/LL)+beta) #LHFE
        joint_goal.append(min(acos((2*Ll*Lu-y**2+2*y*y2)/(-2*Ll*Lu)), 1)) #LKFE
        joint_goal.append(-asin(y2/L)) #LADPF
        joint_goal.append(0) #RHAA
        joint_goal.append(asin(y1/L)) #RHFE
        joint_goal.append(0) #RKFE
        joint_goal.append(asin(y1/L)) #RADPF
        rospy.loginfo(joint_goal)
        plan = self.move_group_right.go(joint_goal,wait=True)
        return plan

def main():
    MoveGroup()