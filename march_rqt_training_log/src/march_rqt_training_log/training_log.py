import os

import rospy
import rospkg
from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class TrainingLogPlugin(Plugin):

    def __init__(self, context):
        super(TrainingLogPlugin, self).__init__(context)
        self.setObjectName('TrainingLogPlugin')

        self._widget = QWidget()
        self.init_ui(context)

        self.training_log_publisher = rospy.Publisher('/march/training/log', String, queue_size=10)

    def init_ui(self, context):
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_training_log'), 'resource', 'training_log.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('TrainingLogUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        self.hideLabel(True)
        self._widget.LogText.undoAvailable.connect(self.hideLabel)
        self._widget.LogButton.clicked.connect(self.log)

    def shutdown_plugin(self):
        rospy.signal_shutdown("rqt plugin closed")

    def log(self):
        message = self._widget.LogText.toPlainText().strip()
        if message:
            rospy.loginfo(message)
            self.training_log_publisher.publish(message)
            self._widget.SuccessLabel.show()
        self._widget.LogText.clear()

    def hideLabel(self, hide):
        if hide:
            self._widget.SuccessLabel.hide()


