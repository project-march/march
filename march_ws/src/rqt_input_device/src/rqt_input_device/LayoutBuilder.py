import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QPushButton
from rqt_input_device.MarchButton import MarchButton



class LayoutBuilder:



    def __init__(self, button_layout):
        self.button_layout = button_layout

    def build(self):
        qt_button_layout = QGridLayout()
        for i in range(len(self.button_layout)):
            qt_button_layout.setRowStretch(i, 1)
            for j in range(len(self.button_layout[i])):
                qt_button_layout.setRowStretch(i, 1)
                march_button = self.button_layout[i][j]
                print march_button
                if march_button is not None:
                    qt_button = self.create_qt_button(march_button)
                else:
                    qt_button = self.create_qt_default_button()
                qt_button_layout.addWidget(qt_button, i, j)

        return qt_button_layout

    def create_qt_button(self, march_button):
        qt_button = QPushButton(march_button.text)
        if march_button.callback is not None:
            qt_button.clicked.connect(march_button.callback)
        qt_button.setStyleSheet(self.create_button_css(self.get_image_path(march_button.image)))
        qt_button.setFixedSize(march_button.size)

        return qt_button

    def create_qt_default_button(self):
        qt_button = QPushButton()
        qt_button.setStyleSheet(self.get_empty_css())
        qt_button.setFixedSize(MarchButton.default_size)
        qt_button.setVisible(False)
        return qt_button

    def get_empty_css(self):
        return """
        display: hidden;
        """

    def create_button_css(self, img_path):
        css_base = """
        background: url(<img_path>) no-repeat center center;
        background-color:#1F1E24;
        """
        return css_base.replace("<img_path>", img_path)

    def get_image_path(self, img_name):
        return os.path.join(rospkg.RosPack().get_path('rqt_input_device'), 'resource', 'img') + img_name
