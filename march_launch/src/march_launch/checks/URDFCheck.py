from march_launch.Color import Color
from LaunchCheck import LaunchCheck
from urdf_parser_py import urdf


class URDFCheck(LaunchCheck):

    def __init__(self):
        LaunchCheck.__init__(self, "URDF", "Please check if the loaded joints are correct", "march_launch",
                             "launch/upload_march_iv_urdf.launch", 10, True)

    def perform(self):
        self.launch()

        robot = urdf.Robot.from_parameter_server()

        count = 0
        for joint in robot.joints:
            if joint.type != "fixed":
                count += 1

        self.log("Loaded " + str(count) + " joints", Color.Info)
        for joint in robot.joints:
            if joint.type != "fixed":
                lower = str(round(joint.safety_controller.soft_lower_limit, 4))
                upper = str(round(joint.safety_controller.soft_upper_limit, 4))
                velocity = str(round(joint.limit.velocity, 4))
                effort = str(round(joint.limit.effort, 4))
                msg = joint.name + ": (" + lower + ", " + upper + ") max velocity: " \
                      + velocity + " rad/s max effort " + effort + " IU"
                self.log(msg, Color.Info)

        self.launch_process.shutdown()
        self.done = True
        self.passed = True
