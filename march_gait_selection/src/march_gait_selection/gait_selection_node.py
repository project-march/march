import rospy
from std_srvs.srv import Trigger

from march_shared_classes.exceptions.gait_exceptions import GaitError
from march_shared_resources.srv import ContainsGait, ContainsGaitResponse, SetGaitVersion

from .gait_selection import GaitSelection
from .perform_gait_action import PerformGaitAction

NODE_NAME = 'gait_selection'
GAIT_FILES_MAP_NAME = 'march_gait_files'
GAIT_DIRECTORY_NAME = 'gait'


def set_gait_versions(msg, gait_selection):
    """Sets a new gait version to the gait selection instance.

    :type msg: march_shared_resources.srv.SetGaitVersionRequest
    :type gait_selection: GaitSelection
    :rtype march_shared_resources.srv.SetGaitVersionResponse
    """
    try:
        gait_selection.set_gait_versions(msg.gait, msg.subgaits, msg.versions)
        return [True, '']
    except GaitError as e:
        return [False, str(e)]


def contains_gait(request, gait_selection):
    """
    Checks whether a gait and subgait are loaded.

    :type request: ContainsGaitRequest
    :param request: service request
    :param gait_selection: current loaded gaits
    :return: True when the gait and subgait are loaded
    """
    gait = gait_selection[request.gait]
    if gait is None:
        return ContainsGaitResponse(False)
    for subgait in request.subgaits:
        if gait[subgait] is None:
            return ContainsGaitResponse(False)

    return ContainsGaitResponse(True)


def main():
    rospy.init_node(NODE_NAME)
    gait_package = rospy.get_param('~gait_package', GAIT_FILES_MAP_NAME)
    gait_directory = rospy.get_param('~gait_directory', GAIT_DIRECTORY_NAME)

    gait_selection = GaitSelection(gait_package, gait_directory)
    rospy.loginfo('Gait selection initialized with package {0} of directory {1}'.format(gait_package, gait_directory))

    # Use lambdas to process service calls inline
    rospy.Service('/march/gait_selection/get_version_map', Trigger,
                  lambda msg: [True, str(gait_selection.get_gait_version_map())])

    rospy.Service('/march/gait_selection/set_gait_version', SetGaitVersion,
                  lambda msg: set_gait_versions(msg, gait_selection))

    rospy.Service('/march/gait_selection/get_directory_structure', Trigger,
                  lambda msg: [True, str(gait_selection.scan_directory())])

    rospy.Service('/march/gait_selection/update_default_versions', Trigger,
                  lambda msg: gait_selection.update_default_versions())

    rospy.Service('/march/gait_selection/contains_gait', ContainsGait,
                  lambda msg: contains_gait(msg, gait_selection))

    PerformGaitAction(gait_selection)
    rospy.spin()
