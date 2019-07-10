import rospy
from pyqtgraph.Qt import QtCore

from std_msgs.msg import Time


class PublishAliveThread(QtCore.QThread):

    def __init__(self):
        QtCore.QThread.__init__(self)
        self.allowed_to_run = True
        self.alive_pub = rospy.Publisher(
            'march/input_device/alive', Time, queue_size=10)

    def run(self):
        rate = rospy.Rate(20)
        while self.allowed_to_run:
            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                self.allowed_to_run = False

            self.alive_pub.publish(Time(rospy.Time.now()))

    def stop(self):
        self.allowed_to_run = False
