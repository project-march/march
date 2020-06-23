from math import sqrt

from numpy import float64
from numpy_ringbuffer import RingBuffer
import rospy
from scipy.signal import savgol_filter
import tf2_ros
from visualization_msgs.msg import Marker


class CPCalculator(object):

    def __init__(self, tf_buffer, foot_link):
        self.tf_buffer = tf_buffer
        self.foot_link = foot_link
        self.publisher = rospy.Publisher('/march/cp_marker_' + foot_link, Marker, queue_size=1)
        self.prev_t = rospy.Time.now()

        self.marker = Marker()

        self.marker.header.frame_id = 'world'
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03

        self.g = 9.81  # gravity constant
        self.buffer_size = 25  # number of com's to use in estimation of velocity
        self.polyorder = 4  # order of the polynomial in Saviztky-Golay

        self.com_x_buffer = RingBuffer(capacity=self.buffer_size, dtype=float64)
        self.com_y_buffer = RingBuffer(capacity=self.buffer_size, dtype=float64)

    def calculate_cp(self, com_mark):
        current_time = com_mark.header.stamp
        if current_time is not self.prev_t:

            self.com_x_buffer.append(com_mark.pose.position.x)
            self.com_y_buffer.append(com_mark.pose.position.y)

            time_difference = (current_time - self.prev_t).to_sec()

            # window length should be odd, greater then poly order and greater or equal to to the buffer size
            if len(self.com_x_buffer) <= self.polyorder:
                return self.marker

            window_length = min(self.buffer_size, len(self.com_x_buffer))
            window_length = window_length if window_length % 2 else window_length - 1
            x_dot = savgol_filter(self.com_x_buffer, window_length=window_length, polyorder=self.polyorder, deriv=1,
                                  delta=time_difference, mode='interp')[-1]
            y_dot = savgol_filter(self.com_y_buffer, window_length=window_length, polyorder=self.polyorder, deriv=1,
                                  delta=time_difference, mode='interp')[-1]

            try:
                trans = self.tf_buffer.lookup_transform('world', self.foot_link, rospy.Time())
                try:
                    multiplier = sqrt(com_mark.pose.position.z / self.g)
                except ValueError:
                    rospy.logdebug_throttle(1, 'Cannot calculate capture point, because center of mass height is '
                                               'smaller than 0')
                    return self.marker

                x_cp = trans.transform.translation.x + x_dot * multiplier
                y_cp = trans.transform.translation.y + y_dot * multiplier

                self.update_marker(x_cp, y_cp)

                self.prev_t = current_time

            except tf2_ros.TransformException as e:
                rospy.logdebug('Error in trying to lookup transform for capture point: {error}'.format(error=e))

        return self.marker

    def update_marker(self, x_cp, y_cp):
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.pose.position.x = x_cp
        self.marker.pose.position.y = y_cp
        self.marker.pose.position.z = 0

        self.publisher.publish(self.marker)
