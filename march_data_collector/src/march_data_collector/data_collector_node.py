import errno
from math import pi
import socket
import sys

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import TransformStamped
import numpy
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker

from march_shared_resources.msg import JointValues, PressureSole


from .com_calculator import CoMCalculator
from .cp_calculator import CPCalculator


class DataCollectorNode(object):
    def __init__(self, com_calculator, cp_calculators, tf_buffer, feet):
        self.differentiation_order = 2
        self._com_calculator = com_calculator
        self._cp_calculators = cp_calculators
        self.tf_buffer = tf_buffer
        self.feet = feet

        self.position_memory = []
        self.time_memory = []
        self.joint_values = JointValues()

        self.joint_values_publisher = rospy.Publisher('/march/joint_values', JointValues, queue_size=1)

        self._imu_broadcaster = tf2_ros.TransformBroadcaster()
        self._com_marker_publisher = rospy.Publisher('/march/com_marker', Marker, queue_size=1)

        self._trajectory_state_subscriber = rospy.Subscriber('/march/controller/trajectory/state',
                                                             JointTrajectoryControllerState,
                                                             self.trajectory_state_callback)

        self._imu_subscriber = rospy.Subscriber('/march/imu', Imu, self.imu_callback)

        self.pressure_soles_on = rospy.get_param('~pressure_soles')

        self.transfrom_imu = TransformStamped()

        self.transfrom_imu.header.frame_id = 'world'
        self.transfrom_imu.child_frame_id = 'imu_link'
        self.transfrom_imu.transform.translation.x = 0.0
        self.transfrom_imu.transform.translation.y = 0.0

        if self.pressure_soles_on:
            rospy.logdebug('will run with pressure soles')
            self.output_host = rospy.get_param('~moticon_ip')
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect(('8.8.8.8', 53))
            self.input_host = sock.getsockname()[0]
            sock.close()
            self.output_port = 8888
            self.output_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.input_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.input_sock.bind((self.input_host, 9999))
            except socket.error:
                rospy.logwarn('Cannot connect to host, is the adress correct? \nrunning without pressure soles')
                self.pressure_soles_on = False
                self.close_sockets()

            self._pressure_sole_publisher = rospy.Publisher('/march/pressure_soles', PressureSole, queue_size=1)
        else:
            rospy.logdebug('running without pressure soles')

    def trajectory_state_callback(self, data):
        com = self._com_calculator.calculate_com()
        self._com_marker_publisher.publish(com)
        for cp_calculator in self._cp_calculators:
            cp_calculator.calculate_cp(com)
        if self.pressure_soles_on:
            self.send_udp(data.actual.positions)

        self.position_memory.append(data.actual.positions)
        self.time_memory.append(data.header.stamp.secs + data.header.stamp.nsecs * 10**(-9))
        if len(self.position_memory) > self.differentiation_order + 1:
            self.position_memory.pop(0)
            self.time_memory.pop(0)
        if len(self.position_memory) > self.differentiation_order:
            velocity = numpy.gradient(self.position_memory, self.time_memory, edge_order=self.differentiation_order,
                                      axis=0)
            acceleration = numpy.gradient(velocity, self.time_memory, edge_order=self.differentiation_order, axis=0)
            jerk = numpy.gradient(acceleration, self.time_memory, edge_order=self.differentiation_order, axis=0)

            self.joint_values.controller_output = data
            self.joint_values.velocities = velocity[-1]
            self.joint_values.accelerations = acceleration[-1]
            self.joint_values.jerks = jerk[-1]

            self.joint_values_publisher.publish(self.joint_values)

    def imu_callback(self, data):
        if data.header.frame_id == 'imu_link':

            z_diff = float('-inf')
            try:
                old_z = self.tf_buffer.lookup_transform('world', 'imu_link', rospy.Time()).transform.translation.z
                for foot in self.feet:
                    trans = self.tf_buffer.lookup_transform('world', foot, rospy.Time())
                    z_diff = max(z_diff, old_z - trans.transform.translation.z)

                self.transfrom_imu.header.stamp = rospy.Time.now()
                self.transfrom_imu.transform.translation.z = z_diff

                imu_rotation = quaternion_multiply([-data.orientation.x, -data.orientation.y, data.orientation.z,
                                                    data.orientation.w], quaternion_from_euler(0, -0.5 * pi, 0))
                self.transfrom_imu.transform.rotation.x = imu_rotation[0]
                self.transfrom_imu.transform.rotation.y = imu_rotation[1]
                self.transfrom_imu.transform.rotation.z = imu_rotation[2]
                self.transfrom_imu.transform.rotation.w = imu_rotation[3]

                self._imu_broadcaster.sendTransform(self.transfrom_imu)
            except tf2_ros.TransformException as e:
                rospy.logdebug('Cannot calculate imu transform, because tf frames are not available, {0}'.format(e))

    def send_udp(self, data):
        message = ' '.join([str(180 * val / pi) for val in data])
        self.output_sock.sendto(message.encode('utf-8'), (self.output_host, self.output_port))

    def receive_udp(self):
        try:
            data, addr = self.input_sock.recvfrom(1024)
            datachannels = data.split()
            values = [float(x) for x in datachannels]
            pressure_sole_msg = PressureSole()
            pressure_sole_msg.header.stamp = rospy.Time.now()
            pressure_sole_msg.pressure_soles_time = rospy.Time(values[0])
            pressure_sole_msg.cop_left = values[1:3]
            pressure_sole_msg.pressure_left = values[3:19]
            pressure_sole_msg.total_force_left = values[19]
            pressure_sole_msg.cop_right = values[20:22]
            pressure_sole_msg.pressure_right = values[22:38]
            pressure_sole_msg.total_force_right = values[38]
            self._pressure_sole_publisher.publish(pressure_sole_msg)
        except socket.timeout:
            rospy.loginfo('Has not received pressure sole data in a while, are they on?')
        except socket.error as error:
            if error.errno == errno.EINTR:
                self.close_sockets()
            else:
                raise
        return

    def close_sockets(self):
        self.output_sock.close()
        self.input_sock.close()

    def run(self):
        if self.pressure_soles_on:
            while not rospy.is_shutdown() and self.pressure_soles_on:
                self.receive_udp()
        else:
            rospy.spin()


def main():
    rospy.init_node('data_collector', anonymous=True)
    try:
        robot = URDF.from_parameter_server('/robot_description')
    except KeyError:
        rospy.logerr('Cannot retrieve URDF from parameter server.')
        sys.exit()
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    center_of_mass_calculator = CoMCalculator(robot, tf_buffer)
    feet = ['foot_left', 'foot_right']
    cp_calculators = [CPCalculator(tf_buffer, foot) for foot in feet]
    data_collector_node = DataCollectorNode(center_of_mass_calculator, cp_calculators, tf_buffer, feet)
    data_collector_node.run()
