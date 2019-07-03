import rospy

from checks.DefaultCheck import DefaultCheck
from Color import Color


class CheckRunner:
    def __init__(self, logger=None):
        self.checks = []
        self.checks.append(DefaultCheck())
        self.logger = logger

    def run_check_by_name(self, name):
        check = self.get_check(name)
        if check is None:
            self.log("Check with name " + name + " does not exist", Color.Error)

        return self.run_check(check)

    def run_check(self, check):
        if check is None:
            self.log("Check does not exist", Color.Error)
            return

        self.log("Starting check " + str(check.name) + ": " + str(check.description), Color.Info)

        start = rospy.get_rostime()
        check.perform()

        while not check.done:
            if rospy.get_rostime() < start + rospy.Duration.from_sec(check.timeout):
                rospy.sleep(0.1)
            else:
                self.log("Check " + str(check.name) + " timed out after " + str(check.timeout/1000.0) + "s", Color.Error)
                check.reset()
                return False

        result = check.passed
        check.reset()

        if result:
            self.log("Check " + str(check.name) + " was succesful!", Color.Debug)
        else:
            self.log("Check " + str(check.name) + " has failed", Color.Error)

        self.log("--------------------------------------", Color.Info)

        return result

    def get_check(self, name):
        for check in self.checks:
            if check.name == name:
                return check
        return None

    def log(self, msg, level):
        if self.logger is not None:
            self.logger(msg, level)
