import ast
import os
import sys

import rospy
import rospkg
from march_shared_resources.srv import StringTrigger, Trigger

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QGroupBox
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QLayout
from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QGridLayout


class GaitSelectionPlugin(Plugin):

    def __init__(self, context):
        super(GaitSelectionPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GaitSelectionPlugin')
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

        # Connect to services
        rospy.wait_for_service('/march/gait_selection/get_version_map', 3)
        self.get_version_map = rospy.ServiceProxy('/march/gait_selection/get_version_map', Trigger)
        rospy.wait_for_service('/march/gait_selection/get_directory_structure', 3)
        self.get_directory_structure = rospy.ServiceProxy('/march/gait_selection/get_directory_structure', Trigger)
        rospy.wait_for_service('/march/gait_selection/set_version_map', 3)
        self.set_version_map = rospy.ServiceProxy('/march/gait_selection/set_version_map', StringTrigger)
        rospy.wait_for_service('/march/gait_selection/update_default_versions', 3)
        self.update_default_versions = rospy.ServiceProxy('/march/gait_selection/update_default_versions', Trigger)

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_selection'), 'resource', 'gait_selection.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Gait Selection')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.refresh()

        self._widget.findChild(QPushButton, "Refresh").clicked.connect(self.refresh)
        self._widget.findChild(QPushButton, "Apply").clicked.connect(
            lambda: [
                self.set_gait_selection_map(),
                self.refresh()
            ])
        self._widget.findChild(QPushButton, "SaveDefault").clicked.connect(
            lambda: [
                self.set_gait_selection_map(),
                self.update_default_versions(),
                self.refresh()
            ])

    def refresh(self):
        rospy.logdebug("Refreshing ui with %s", str(self.get_directory_structure().message))

        try:
            gait_version_map = ast.literal_eval(self.get_version_map().message)
        except ValueError:
            rospy.logerr("Gait selection map is not valid %s" + str(self.get_version_map().message))
            return
        try:
            gait_directory_structure = ast.literal_eval(self.get_directory_structure().message)
        except ValueError:
            rospy.logerr("Gait directory structure is not valid %s" + str(self.get_directory_structure().message))
            return


        gaits = self._widget.Gaits.findChildren(QGroupBox, "Gait")
        for gait in gaits:
            gait.deleteLater()

        self.load_ui(gait_directory_structure, gait_version_map)
        self._widget.layout().setSizeConstraint(QLayout.SetFixedSize)

    def load_ui(self, gait_directory_structure, gait_selection_map):
        for gait_name, gait in gait_directory_structure.iteritems():
            try:
                selection_map = gait_selection_map[gait_name]
            except KeyError:
                selection_map = None
            gait_group_box = self.create_gait(gait_name, gait, selection_map)
            self._widget.Gaits.layout().addWidget(gait_group_box)

    def create_gait(self, name, gait, selections):
        gait_group_box = QGroupBox()
        gait_group_box.setObjectName("Gait")
        gait_group_box.setLayout(QHBoxLayout())

        gait_group_box.setTitle(name)
        image = QLabel()
        image.setStyleSheet(
            "background: url(" + gait["image"] + ") no-repeat center center 100px 100px;")
        image.setFixedSize(64, 80)
        gait_group_box.layout().addWidget(image)
        for subgait_name, subgait in gait["subgaits"].iteritems():
            subgait_group_box = self.create_subgait(subgait_name, subgait, selections)
            gait_group_box.layout().addWidget(subgait_group_box)

        return gait_group_box

    def create_subgait(self, name, subgait, version_selection):
        subgait_group_box = QGroupBox()
        subgait_group_box.setLayout(QGridLayout())
        subgait_group_box.setObjectName("Subgait")
        subgait_group_box.setTitle(name)
        try:
            version_name = version_selection[name]
        except TypeError:
            version_name = None
        except KeyError:
            version_name = None
        dropdown = self.create_dropdown(subgait, version_name)

        subgait_group_box.layout().addWidget(dropdown, 0, 0)
        return subgait_group_box

    def create_dropdown(self, options, selection):
        try:
            index = options.index(selection)
        except ValueError:
            rospy.logwarn("Selection %s not found in options %s.", str(selection), str(options))
            index = -1
        dropdown = QComboBox()
        for option in options:
            dropdown.addItem(option)
        dropdown.setCurrentIndex(index)
        return dropdown

    def set_gait_selection_map(self):
        gait_selection_map = {}
        gaits = self._widget.Gaits.findChildren(QGroupBox, "Gait")
        for gait in gaits:
            gait_name = str(gait.title())
            gait_selection_map[gait_name] = {}

            subgaits = gait.findChildren(QGroupBox, "Subgait")
            for subgait in subgaits:
                subgait_name = str(subgait.title())
                version = str(subgait.findChild(QComboBox).currentText())
                gait_selection_map[gait_name][subgait_name] = version

        res = self.set_version_map(str(gait_selection_map))
        if res.success:
            rospy.loginfo(res.message)
        else:
            rospy.logwarn(res.message)

    """Return all widgets found in the requested layout."""
    def get_layout_widgets(self, layout):
        return (layout.itemAt(i).widget() for i in range(layout.count()))
