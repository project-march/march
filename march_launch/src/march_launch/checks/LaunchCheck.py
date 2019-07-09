import os

import rospkg

from SoftwareCheck import SoftwareCheck
import roslaunch


class LaunchCheck(SoftwareCheck):

    def __init__(self, name, description, package, file, timeout=10000, manual_confirmation=False):
        SoftwareCheck.__init__(self, name, description, timeout, manual_confirmation)
        self.package = package
        self.file = file
        self.launch_process = None

    def launch(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = os.path.join(rospkg.RosPack().get_path(self.package), self.file)

        self.launch_process = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self.launch_process.start()

    def perform(self):
        raise NotImplementedError("Please implement method 'perform()' on the subclass")
