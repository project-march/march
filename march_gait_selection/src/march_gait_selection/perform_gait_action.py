import actionlib
import rospy

from march_gait_selection.dynamic_gaits.transition_subgait import TransitionSubgait
from march_shared_classes.exceptions.general_exceptions import MsgTypeError
from march_shared_resources import msg
from march_shared_resources.msg import GaitAction, GaitGoal, GaitNameAction

SERVER_TIMEOUT = 5
RESPONSE_TIMEOUT = 1


class PerformGaitAction(object):
    def __init__(self, gait_selection):
        self.gait_selection = gait_selection
        self.action_server = actionlib.SimpleActionServer('/march/gait/perform', GaitNameAction,
                                                          execute_cb=self.target_gait_callback,
                                                          auto_start=False)
        self.action_server.start()
        self.schedule_gait_client = actionlib.SimpleActionClient('/march/gait/schedule', GaitAction)

        while not rospy.is_shutdown() and not self.schedule_gait_client.wait_for_server(rospy.Duration(SERVER_TIMEOUT)):
            rospy.logdebug('Waiting for /march/gait/schedule to come up')

    def target_gait_callback(self, subgait_goal_msg):
        """Set a new target subgait over the action server march/gait/schedule."""
        rospy.logdebug('Trying to schedule subgait {gn} {sn}'
                       .format(gn=subgait_goal_msg.name, sn=subgait_goal_msg.subgait_name))

        gait = self.gait_selection[subgait_goal_msg.name]
        if gait:
            subgait = gait[subgait_goal_msg.subgait_name]
            if subgait:
                if subgait_goal_msg.old_name:
                    old_gait_name = subgait_goal_msg.old_name
                    gait_name = subgait_goal_msg.name
                    subgait_name = subgait_goal_msg.subgait_name
                    rospy.logdebug('Create with old gait: {og}, gait: {ng}, subgait: {sg}'
                                   .format(og=old_gait_name, ng=gait_name, sg=subgait_name))
                    subgait = TransitionSubgait.from_subgait_names(self.gait_selection, old_gait_name,
                                                                   gait_name, subgait_name)
                if not isinstance(subgait, msg.Subgait):
                    subgait = subgait.to_subgait_msg()
                trajectory_state = self.schedule_gait(subgait_goal_msg.name, subgait)

                if trajectory_state == actionlib.GoalStatus.SUCCEEDED:
                    self.action_server.set_succeeded(trajectory_state)
                else:
                    self.action_server.set_aborted(trajectory_state)

                return True

        rospy.logwarn('Gait {gn} with subgait {sn} does not exist in parsed gaits'
                      .format(gn=subgait_goal_msg.name, sn=subgait_goal_msg.subgait_name))

        self.action_server.set_aborted('Gait {gn} with subgait {sn} does not exist in parsed gaits'
                                       .format(gn=subgait_goal_msg.name, sn=subgait_goal_msg.subgait_name))

        return False

    def schedule_gait(self, gait_name, subgait):
        """Construct the goal message and send."""
        if not isinstance(subgait, msg.Subgait):
            raise MsgTypeError(msg='Given subgait is not of type msg.Subgait')

        gait_action_goal = GaitGoal()
        gait_action_goal.name = gait_name
        gait_action_goal.current_subgait = subgait

        self.schedule_gait_client.send_goal(gait_action_goal)
        self.schedule_gait_client.wait_for_result(timeout=subgait.duration + rospy.Duration(RESPONSE_TIMEOUT))

        return self.schedule_gait_client.get_state()
