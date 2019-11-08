#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Temperature
from control_msgs.msg import JointTrajectoryControllerState
from march_shared_resources.msg import ImcErrorState


def TemperatureCallback(data, joint):
    rospy.logdebug('Temperature' + joint + ' is ' + str(data.temperature))


def TrajectoryStateCallback(data):
    rospy.logdebug('received trajectory state' + str(data.desired))


def ImcStateCallback(data):
    rospy.logdebug('received IMC message current is ' + str(data.current))


def main():
    rospy.init_node('data_collector', anonymous=True)
    joint_names = rospy.get_param('/march/joint_names')

    TemperatureSubscriber = [rospy.Subscriber('/march/temperature/'+joint, Temperature, TemperatureCallback,
                                              (joint)) for joint in joint_names]

    TrajectoryStateSubscriber = rospy.Subscriber('/march/controller/trajectory/state', JointTrajectoryControllerState,
                                                 TrajectoryStateCallback)

    IMCStateSubscriber = rospy.Subscriber('/march/imc_states', ImcErrorState, ImcStateCallback)

    rospy.spin()
