import smach
import rospy
from march_custom_msgs.srv import PerformGait


class GaitState(smach.State):
    def __init__(self, gait_name, gait_dict, play_dict, outcomes):
        self.gait_name = gait_name
        self.gait_dict = gait_dict
        self.play_dict = play_dict
        smach.State.__init__(self, outcomes=outcomes)

    def execute(self, userdata):
        rospy.loginfo('Trying to perform ' + self.gait_name)
        perform_gait_service = rospy.ServiceProxy('march/perform_gait', PerformGait)
        result = perform_gait_service(self.gait_name)
        rospy.loginfo(self.gait_name + ' performed')

        rospy.Subscriber("play_input", Empty, callback)
        rospy.Subscriber("gait_input", Gait, callback)

        rospy.spin()
        if result.success:
            return 'succeeded'
        else:
            return 'failed'
