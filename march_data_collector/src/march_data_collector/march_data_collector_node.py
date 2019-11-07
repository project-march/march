#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from control_msgs.msg import JointTrajectoryControllerState
from march_shared_resources.msg import ImcErrorState


def TemperatureCallback(data, joint):
    rospy.logdebug('Temperature' + joint + ' is ' + str(data.temperature))


def TrajectoryStateCallback(data):
    rospy.logdebug('received trajectory state' + str(data.desired))


def ImcStateCallback(data):
    rospy.logdebug('received IMC message current is ' + str(data.current))

def IMUCallback(data):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "hip_base"
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    pose.pose.orientation.x = data.orientation[0]
    pose.pose.orientation.y = data.orientation[1]
    pose.pose.orientation.z = data.orientation[2]
    pose.pose.orientation.w = data.orientation[3]

    PosePublisher.publish(pose)


def main():
    rospy.init_node('data_collector', anonymous=True)
    joint_names = rospy.get_param('/march/joint_names')
    PosePublisher = rospy.Publisher('/pose',PoseStamped)

    TemperatureSubscriber = [rospy.Subscriber('/march/temperature/'+joint, Temperature, TemperatureCallback,
                                              (joint)) for joint in joint_names]

    TrajectoryStateSubscriber = rospy.Subscriber('/march/controller/trajectory/state', JointTrajectoryControllerState,
                                                 TrajectoryStateCallback)

    IMCStateSubscriber = rospy.Subscriber('/march/imc_states', ImcErrorState, ImcStateCallback)

    IMUSubsriber = rospy.Subscriber('/march/imu/00B447AD', Imu, IMUCallback)

    rospy.spin()
