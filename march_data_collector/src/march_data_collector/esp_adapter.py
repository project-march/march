# This file contains ESP adapter class which sends data comming on ROS topic to an already running ESP enginge.
# It is assumed that the pubsub is opened on port 9901. The project is March_test and the continious query is March_cq.
# The connection is establish trough the libraries modelingApi.py and pubsubApi.py, which come with the installation and
# are a wrapper for the C Api. Detailed documentation about ESP can be found at
# https://documentation.sas.com/?cdcId=espcdc&cdcVersion=6.2&docsetId=espov&docsetTarget=home.htm&locale=nl

import datetime
import logging
import os
import sys

from control_msgs.msg import PidState
import rospy
from sensor_msgs.msg import Imu, Temperature
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

from march_shared_resources.msg import (AfterLimitJointCommand, CurrentGait, CurrentState, ImcState, JointValues,
                                        PressureSole)

try:
    sys.path.append(os.environ['DFESP_HOME'] + '/lib')
    import pubsubApi
    import modelingApi
except (ImportError, KeyError) as e:
    rospy.logerr('Error while loading libraries for ESP.\nProbably ESP is not installed on this machine.')
    sys.exit()


class ESPAdapter:
    """Class streams ros messages into an ESP engine."""

    def __init__(self):

        try:
            joint_names = rospy.get_param('/march/joint_names')
        except KeyError:
            joint_names = ['left_hip_aa', 'left_hip_fe', 'left_knee', 'left_ankle', 'right_hip_aa',
                           'right_hip_fe', 'right_knee', 'right_ankle']
            rospy.loginfo('Cannot get joint_names from parameter server assuming usual 8: \n' + str(joint_names))

        ret = pubsubApi.Init(modelingApi.ll_Off, None)
        if ret == 0:
            rospy.logerr('Could not initialize pubsub library. \nKilling ESP adapater.')
            sys.exit()

        logger = logging.getLogger()
        logger.addHandler(modelingApi.getLoggingHandler())

        self.esp_publishers = {}
        self.ros_subscribers = {}

        self.previous_join_key = {}

        self.control_analysis_join_frequency = 50

        basic_url = 'dfESP://localhost:9901'
        self.project = basic_url + '/project_march'
        stringv = pubsubApi.QueryMeta(self.project + '?get=windows_sourceonly')
        if stringv is None:
            projects_ptr = pubsubApi.QueryMeta(basic_url + '?get=projects')
            if projects_ptr is None:
                rospy.logerr('Cannot connect to ESP server, is it running?\nKilling ESP adapter.')
            else:
                queries_ptr = pubsubApi.QueryMeta(self.project + '?get=projects')
                if queries_ptr is None:
                    rospy.logerr('Cannot connect to the desired project on the ESP server.\n killing ESP adapter')
                    rospy.loginfo('Possible projects are:\n' + str(convert_stringv(projects_ptr, True)))
                else:
                    rospy.logerr('Cannot connect to the desired continious query on the ESP server.\n'
                                 'Killing ESP adapter.')
                    rospy.loginfo('Possible continious queries are:\n' + str(convert_stringv(queries_ptr, True)))
            sys.exit()

        self.source_windows_esp = set(convert_stringv(stringv, True))

        for joint in joint_names:
            self.configure_source(['temperature_join/source_temperature_' + joint], '/march/temperature/' + joint,
                                  Temperature, self.temperature_callback)

        for joint in joint_names:
            self.configure_source(['control_analysis/source_pid_state_' + joint], '/march/controller/trajectory/gains/'
                                  + joint + '/state', PidState, self.pid_state_callback)

        self.configure_source(['gait_analysis/source_imu'], '/march/imu', Imu, self.imu_callback)

        self.configure_source(['gait_analysis/source_ps'], '/march/pressure_soles', PressureSole,
                              self.pressure_sole_callback)
        self.configure_source(['control_analysis/source_imc'], '/march/imc_states', ImcState, self.imc_state_callback)
        self.configure_source(['control_analysis/source_gait_control', 'gait_analysis/source_gait'],
                              '/march/gait_selection/current_gait', CurrentGait, self.gait_callback)
        self.configure_source(['control_analysis/source_gait_control', 'gait_analysis/source_gait'],
                              '/march/gait_selection/current_state', CurrentState, self.gait_finished_callback)
        self.configure_source(['gait_analysis/source_com'], '/march/com_marker', Marker, self.com_callback)
        self.configure_source(['gait_analysis/source_joint'], '/march/joint_values', JointValues,
                              self.joint_values_callback)
        self.configure_source(['control_analysis/source_effort_command'],
                              '/march/controller/after_limit_joint_command', AfterLimitJointCommand,
                              self.joint_command_callback)

    def configure_source(self, sources, topic, msg_type, callback):
        """Configures a connection between a ROS topic and a source window in an event stream processing engine.

        :param sources: list of source windows in the ESP engine
        :param topic: the topic on which the ROS messages are posted
        :param msg_type: the type of the ROS messages
        :param callback: the callback function that can receive the ROS message and create a CSV string for the
        source window
        """
        sub = rospy.Subscriber(topic, msg_type, callback, sources)
        for source in sources:
            if source.split('/')[1] not in self.source_windows_esp:
                rospy.logwarn('There is no ESP source window for the following source: ' + source)
                break

            window_url = '/'.join([self.project, source])
            stringv = pubsubApi.QueryMeta(window_url + '?get=schema')

            if stringv is None:
                rospy.logwarn('Could not get ESP source window schema for window ' + source)
                modelingApi.StringVFree(stringv)
                break

            schema = modelingApi.StringVGet(stringv, 0)

            if schema is None:
                rospy.logwarn('Could not get ESP schema from query response for source ' + source)
                break

            schemaptr = modelingApi.SchemaCreate(source, schema)
            if schemaptr is None:
                rospy.logwarn('Could not build ESP source window schema for source ' + source)
                break

            pub = pubsubApi.PublisherStart(window_url, pubsubApi.ERRCBFUNC(pub_err_cb_func), None)
            if pub is None:
                rospy.logwarn('Could not create ESP publisher client for source' + source)
                break

            ret = pubsubApi.Connect(pub)
            if ret != 1:
                rospy.logwarn('Could not connect ESP publisher client for source ' + source)
                break

            self.esp_publishers[source] = (pub, schemaptr)
            rospy.logdebug('configured ESP source window for ' + source)
            self.ros_subscribers[source] = sub
        self.previous_join_key[sources[0]] = None

    def send_to_esp(self, csv, sources):
        """Sends a csv string to the configured source window. Also adds standard stuff to the start of the csv string.

        :param csv: the csv string containing all data to send to the message
        :param sources: list of source windows in the ESP engine
        """
        csv = 'i, n, ' + csv
        for source in sources:
            try:
                pub, schemaptr = self.esp_publishers[source]
            except KeyError:
                rospy.loginfo_throttle(3, 'Receiving data for ' + source + ', but cannot send to ESP, because the '
                                                                           'source window is not configured.')
                rospy.logwarn(csv)
                break
            event = modelingApi.EventCreate2(schemaptr, csv, '%Y-%m-%d %H:%M:%S')
            event_vector = modelingApi.EventVCreate()
            modelingApi.EventVPushback(event_vector, event)
            event_block = modelingApi.EventBlockNew1(event_vector, modelingApi.ebt_NORMAL)
            ret = pubsubApi.PublisherInject(pub, event_block)
            modelingApi.EventBlockDestroy(event_block)
            if ret != 1:
                rospy.logwarn('Unsuccessful inject into ESP server for source window {source} and event {event}'.format(
                    source=source, event=csv))

    def gait_finished_callback(self, data, sources):
        """Callback for stopped gait. If the current state is an idle state send this to ESP a gait.

        :param data: ROS message
        :param sources: list of source windows in the ESP engine
        """
        if data.state_type == CurrentState.IDLE:
            csv = ','.join([get_time_str(data.header.stamp), 'idle', data.state.lower(), ' ', ' '])
            self.send_to_esp('1, 1, {0}'.format(csv), sources)

    def joint_command_callback(self, data, sources):
        """Callback for after_limit_joint_command data. Converts ROS message to csv string to send to the source window.

        :param data: ROS temperature message
        :param sources: the name of the source window in the ESP engine
        """
        time_str = get_time_str(data.header.stamp)
        join_time_str = get_join_time_str(data.header.stamp, self.control_analysis_join_frequency)

        if self.previous_join_key[sources[0]] == join_time_str:
            return
        self.previous_join_key[sources[0]] = join_time_str

        csv = list_to_str(['1', join_time_str, data.header.seq, time_str, list_to_str(data.effort_command)])
        self.send_to_esp(csv, sources)

    def temperature_callback(self, data, sources):
        """Callback for temperature data. Converts ROS message to csv string to send to the source window.

        :param data: ROS temperature message
        :param sources: list of source windows in the ESP engine
        """
        self.send_to_esp('{0}, {1}'.format(get_time_str(data.header.stamp), data.temperature), sources)

    def joint_values_callback(self, data, sources):
        """Callback for trajectory_state data. Converts ROS message to csv string to send to the source window.

        :param data: ROS march_shared_resources.msgs.JointValues
        :param sources: list of source windows in the ESP engine
        """
        actual_positions_str = list_to_str(data.controller_output.actual.positions)
        actual_velocity_str = list_to_str(data.velocities)
        actual_acceleration_str = list_to_str(data.accelerations)
        actual_jerk_str = list_to_str(data.jerks)
        desired_positions_str = list_to_str(data.controller_output.desired.positions)
        desired_velocity_str = list_to_str(data.controller_output.desired.velocities)
        position_error_str = list_to_str(data.controller_output.error.positions)
        time_str = get_time_str(data.controller_output.header.stamp)

        csv = list_to_str([data.controller_output.header.seq, '1', time_str, actual_positions_str, actual_velocity_str,
                           actual_acceleration_str, actual_jerk_str, desired_positions_str, desired_velocity_str,
                           position_error_str])
        self.send_to_esp(csv, sources)

    def imu_callback(self, data, sources):
        """Callback for imu data. Converts ROS message to csv string to send to the source window.

        :param data: ROS sensor_msgs.Imu message
        :param sources: list of source windows in the ESP engine
        """
        orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        orientation_str = list_to_str(euler_from_quaternion(orientation))

        angular_velocity_str = vector_to_str(data.angular_velocity)
        linear_acceleration_str = vector_to_str(data.linear_acceleration)
        time_str = get_time_str(data.header.stamp)

        csv = list_to_str([data.header.seq, '1', time_str, orientation_str, angular_velocity_str,
                           linear_acceleration_str])
        self.send_to_esp(csv, sources)

    def pid_state_callback(self, data, sources):
        """Callback for controller data. Converts ROS message to csv string to send to the source window.

        :param data: ROS PidState message
        :param sources: list of source windows in the ESP engine
        """
        time_str = get_time_str(data.header.stamp)
        join_time_str = get_join_time_str(data.header.stamp, self.control_analysis_join_frequency)

        if self.previous_join_key[sources[0]] == join_time_str:
            return
        self.previous_join_key[sources[0]] = join_time_str

        csv = list_to_str([data.p_error, data.i_error, data.d_error, data.p_term, data.i_term,
                           data.d_term, data.output])
        self.send_to_esp('{0}, {1}, {2}, 1'.format(time_str, join_time_str, csv), sources)

    def imc_state_callback(self, data, sources):
        """Callback for IMotionCube data. Converts ROS message to csv string to send to the source window.

        :param data: ROS march_shared_resources.msgs.ImcErrorState message
        :param sources: list of source windows in the ESP engine
        """
        time_str = get_time_str(data.header.stamp)
        join_time_str = get_join_time_str(data.header.stamp, self.control_analysis_join_frequency)

        if self.previous_join_key[sources[0]] == join_time_str:
            return
        self.previous_join_key[sources[0]] = join_time_str

        motor_current_str = ','.join([str(value) for value in data.motor_current])
        imc_voltage_str = ','.join([str(value) for value in data.imc_voltage])
        absolute_encoder_str = ','.join([str(value) for value in data.absolute_encoder_value])
        incremental_encoder_str = ','.join([str(value) for value in data.incremental_encoder_value])
        motor_voltage_str = ','.join([str(value) for value in data.motor_voltage])
        absolute_velocity_str = ','.join([str(value) for value in data.absolute_velocity])
        incremental_velocity_str = ','.join([str(value) for value in data.incremental_velocity])
        csv = ','.join([time_str, motor_current_str, imc_voltage_str, motor_voltage_str, absolute_encoder_str,
                        incremental_encoder_str, absolute_velocity_str, incremental_velocity_str])
        self.send_to_esp('{0}, {1}, 1, {2}'.format(join_time_str, data.header.seq, csv), sources)

    def gait_callback(self, data, sources):
        """Callback for gait data. Converts ROS message to csv string to send to the source window.

        :param data: ROS message
        :param sources: list of source windows in the ESP engine
        """
        csv = list_to_str(['1, 1', get_time_str(data.header.stamp), data.gait, data.subgait,
                           data.version, data.gait_type])
        self.send_to_esp(csv, sources)

    def com_callback(self, data, sources):
        """Callback for center of mass data. Converts ROS message to csv string to send to the source window.

        :param data: ROS visualization_msgs.Marker message
        :param sources: list of source windows in the ESP engine
        """
        csv = ','.join([get_time_str(data.header.stamp), str(data.pose.position.x), str(data.pose.position.y),
                        str(data.pose.position.z)])
        self.send_to_esp('{0}, 1, {1}'.format(data.header.seq, csv), sources)

    def pressure_sole_callback(self, data, sources):
        """Callback for pressure sole data. Converts ROS message to csv string to send to the source window.

        :param data: ROS march_shared_resources.PressureSole message
        :param sources: list of source windows in the ESP engine
        """
        pressure_left = list_to_str(data.pressure_left)
        pressure_right = list_to_str(data.pressure_left)
        cop_left = list_to_str(data.cop_left)
        cop_right = list_to_str(data.cop_right)
        csv = ','.join([get_time_str(data.pressure_soles_time), str(data.total_force_left), str(data.total_force_right),
                        pressure_left, pressure_right, cop_left, cop_right])
        self.send_to_esp('{0}, 1, {1}'.format(data.header.seq, csv), sources)


def get_time_str(timestamp):
    """Creates str to use in csv string for source window based on timestamp.

    :param data: ROS timestamp message std_msgs/stamp
    """
    time = round(timestamp.to_sec(), 3)
    return datetime.datetime.fromtimestamp(time).strftime('%Y-%m-%d %H:%M:%S.%f')


def get_join_time_str(timestamp, frequency):
    """Creates str to use in csv string for source window based on timestamp.

    :param data: ROS timestamp message std_msgs/stamp
    """
    time = round(timestamp.to_sec() * frequency) / frequency
    return datetime.datetime.fromtimestamp(time).strftime('%Y-%m-%d %H:%M:%S.%f')


def list_to_array_str(ls):
    """Converts a list to an array formatted string for source window.

    :type ls: list
    """
    return '[' + ';'.join(str(value) for value in ls) + ']'


def list_to_str(ls):
    """Converts a list to a csv string for source window.

    :type ls: list
    """
    return ', '.join(str(value) for value in ls)


def vector_to_str(vector):
    """Converts geometry_msgs/Vector3 to string to use in csv string for source window.

    :type vector: geometry_msgs/Vector3
    """
    return '{x},{y},{z}'.format(x=vector.x, y=vector.y, z=vector.z)


def convert_stringv(stringv, free):
    """Converts a stringV object (integer pointer, which points to a string vector via de API) to a list.

    :param data: stringV object from the modelingApi to convert
    :param free: whether to free the object afterwards
    """
    ls = [modelingApi.StringVGet(stringv, i) for i in range(0, modelingApi.StringVSize(stringv))]
    if free:
        modelingApi.StringVFree(stringv)
    return ls


def mock_get_gait():
    """Mocks the get_gait ROS service when not available.

    :return: mock CurrentState msg
    """
    msg = CurrentState()
    msg.current_state = 'UNKNOWN'
    msg.state_type = 'idle'
    return msg


def pub_err_cb_func(self, failure, code, _):
    if failure == pubsubApi.pubsubFail_APIFAIL and code == pubsubApi.pubsubCode_CLIENTEVENTSQUEUED:
        return

    fail_msg = pubsubApi.DecodeFailure(failure)
    code_msg = pubsubApi.DecodeFailureCode(code)
    rospy.logerr('Client services error: ' + fail_msg + code_msg)


def main():
    rospy.init_node('esp_adapter', anonymous=True)
    ESPAdapter()
    rospy.spin()
