from math import sqrt

from march_data_collector.inverted_pendulum import InvertedPendulum
import rospy
import tf2_ros
from visualization_msgs.msg import Marker


class CPCalculator(object):

    def __init__(self, tf_buffer, foot_link):
        """Base class to calculate capture point for the exoskeleton."""
        self._tf_buffer = tf_buffer
        self._foot_link = foot_link

        rospy.Service('/march/cp_marker_{fl}'.format(fl=foot_link), Marker, self.get_capture_point)

        self._gravity_constant = 9.81

        self._prev_t = rospy.Time.now()
        self._prev_x = 0
        self._prev_y = 0

        self._center_of_mass_marker = Marker()
        self._capture_point_marker = Marker()

        self._capture_point_marker.header.frame_id = 'world'
        self._capture_point_marker.type = self._capture_point_marker.SPHERE
        self._capture_point_marker.action = self._capture_point_marker.ADD
        self._capture_point_marker.pose.orientation.w = 1.0

        self._capture_point_marker.color.a = 1.0
        self._capture_point_marker.color.g = 1.0

        self._capture_point_marker.scale.x = 0.03
        self._capture_point_marker.scale.y = 0.03
        self._capture_point_marker.scale.z = 0.03

    @property
    def center_of_mass_marker(self):
        """Center of mass property getter."""
        return self._center_of_mass_marker

    @center_of_mass_marker.setter
    def center_of_mass_marker(self, updated_center_of_mass):
        """Center of mass property setter."""
        if not isinstance(updated_center_of_mass, Marker):
            raise TypeError('Given center of mass is not of type; Marker')

        self._center_of_mass_marker = updated_center_of_mass

    def _calculate_capture_point(self, duration):
        """Calculate a future capture point pose using the inverted pendulum and center of mass.

        :param duration:
            the amount of seconds away from the current time the capture point should be calculated
        """
        current_time = self._center_of_mass_marker.header.stamp

        if current_time is not self._prev_t:
            time_difference = (current_time - self._prev_t).to_sec()
            vx = (self._center_of_mass_marker.pose.position.x - self._prev_x) / time_difference
            vy = (self._center_of_mass_marker.pose.position.y - self._prev_y) / time_difference

            x = self._center_of_mass_marker.pose.position.x
            y = self._center_of_mass_marker.pose.position.y
            z = self._center_of_mass_marker.pose.position.z

            future_center_of_mass = InvertedPendulum.numeric_solve_to_t(x, y, z, vx, vy, duration)

            try:
                world_transform = self._tf_buffer.lookup_transform('world', self._foot_link, rospy.Time())
                if future_center_of_mass.pose.position.z <= 0:
                    rospy.logdebug_throttle(1, 'Cannot calculate capture point; center of mass < 0')

                capture_point_multiplier = sqrt(future_center_of_mass.z / self._gravity_constant)

                x_cp = world_transform.transform.translation.x + future_center_of_mass.vx * capture_point_multiplier
                y_cp = world_transform.transform.translation.y + future_center_of_mass.vy * capture_point_multiplier

                self._prev_t = current_time
                self._prev_x = future_center_of_mass.x
                self._prev_y = future_center_of_mass.y

                self._capture_point_marker.header.stamp = rospy.get_rostime()
                self._capture_point_marker.pose.position.x = x_cp
                self._capture_point_marker.pose.position.y = y_cp
                self._capture_point_marker.pose.position.z = 0

            except tf2_ros.TransformException as e:
                rospy.logdebug('Error in trying to lookup transform for capture point: {error}'.format(error=e))

    def get_capture_point(self, duration):
        """Service call function to return the capture point pose.

        :param duration:
            the amount of seconds away from the current time the capture point should be calculated=
        """
        self._calculate_capture_point(duration)
        return self._capture_point_marker
