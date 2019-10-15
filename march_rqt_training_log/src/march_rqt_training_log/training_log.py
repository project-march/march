import os

import rospy
import rospkg
from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QWidget, QShortcut


class TrainingLogPlugin(Plugin):

    def __init__(self, context):
        super(TrainingLogPlugin, self).__init__(context)
        self.setObjectName('TrainingLogPlugin')

        self._widget = QWidget()
        self.init_ui(context)
        self.init_shortcuts()

        self.timestamp = ''
        self.training_log_publisher = rospy.Publisher('/march/training/log', String, queue_size=10)

    def init_ui(self, context):
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_training_log'), 'resource', 'training_log.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('TrainingLogUi')
        context.add_widget(self._widget)

        self.hideLabel(True)
        self._widget.LogText.undoAvailable.connect(self.hideLabel)
        self._widget.LogButton.clicked.connect(self.log)
        self._widget.TimestampButton.clicked.connect(self.create_timestamp)
        self._widget.ClearButton.clicked.connect(self.clear)

    def init_shortcuts(self):
        log_shortcut = QShortcut(QKeySequence('Ctrl+L'), self._widget)
        clear_shortcut = QShortcut(QKeySequence('Ctrl+D'), self._widget)
        timestamp_shortcut = QShortcut(QKeySequence('Ctrl+T'), self._widget)
        log_shortcut.activated.connect(self.log)
        clear_shortcut.activated.connect(self.clear)
        timestamp_shortcut.activated.connect(self.create_timestamp)

    def shutdown_plugin(self):
        rospy.signal_shutdown('rqt plugin closed')

    def log(self):
        if self.get_text():
            message = self.create_message()
            rospy.loginfo(message)
            self.training_log_publisher.publish(message)
            self._widget.SuccessLabel.show()
            self._widget.TimestampLabel.clear()
            self.timestamp = ''
        self._widget.LogText.clear()

    def get_text(self):
        return self._widget.LogText.toPlainText().strip()

    def get_camera_take(self):
        return self._widget.CameraTakeText.text().strip()

    def create_message(self):
        message = ''
        if self.timestamp:
            message += '[%s] ' % self.timestamp
        if self.get_camera_take():
            message += '[take: %s] ' % self.get_camera_take()

        return message + self.get_text()

    def create_timestamp(self):
        self.timestamp = '%.5f' % rospy.get_time()
        rospy.loginfo('timestamped at %s', self.timestamp)
        self._widget.TimestampLabel.setText(self.timestamp)
        self._widget.SuccessLabel.hide()

    def clear(self):
        self.timestamp = ''
        self._widget.TimestampLabel.clear()
        self._widget.LogText.clear()
        self._widget.SuccessLabel.hide()
        self._widget.CameraTakeText.clear()

    def hideLabel(self, hide):
        if hide:
            self._widget.SuccessLabel.hide()
