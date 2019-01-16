import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from march_shared_resources.msg import Gait

class InputDevicePlugin(Plugin):

    def __init__(self, context):
        super(InputDevicePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('InputDevicePlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_input_device'), 'resource', 'input_device.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Input Device')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.instruction_gait = rospy.Publisher('march/input_device/instruction/gait', Gait, queue_size=10)

        self._widget.home_to_sit_button.clicked.connect(self.home_to_sit_cb)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def home_to_sit_cb(self):
        self.instruction_gait.publish(Gait("home_to_sit", 1000))
    #def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog



class TopicPublisher(object):

    def __init__(self, topic_name, message_class):
        self._name = topic_name
        try:
            self._publisher = rospy.Publisher(
                topic_name, message_class, queue_size=100)
            self._message = message_class()
        except ValueError as e:
            rospy.logfatal('msg file for %s not found' % topic_name)
            raise e

    def get_topic_name(self):
        return self._name

    def publish(self):
        self._publisher.publish(self._message)

    def get_message(self):
        return self._message
