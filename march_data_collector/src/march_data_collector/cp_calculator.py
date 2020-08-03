from math import sqrt

from march_data_collector.inverted_pendulum import InvertedPendulum
import rospy
import tf2_ros
from visualization_msgs.msg import Marker

from march_shared_resources.srv import CapturePointPose


class CPCalculator(object):

    def __init__(self, tf_buffer, foot_link):
        """Base class to calculate capture point for the exoskeleton."""
        self._tf_buffer = tf_buffer
        self._foot_link = foot_link

        self.cp_service = rospy.Service('/march/cp_marker_{fl}'.format(fl=foot_link), CapturePointPose,
                                        lambda msg: self.get_capture_point(msg))

        self._gravity_constant = 9.81
        self._prev_t = rospy.Time.now()
        self._delta_t = 0

        self.x = 0
        self.y = 0
        self.z = 0
        self.vx = 0
        self.vy = 0

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

        current_time = updated_center_of_mass.header.stamp
        self._delta_t = (current_time - self._prev_t).to_sec()

        if self._delta_t == 0:
            return

        self.vx = (updated_center_of_mass.pose.position.x - self._center_of_mass_marker.pose.position.x) / self._delta_t
        self.vy = (updated_center_of_mass.pose.position.y - self._center_of_mass_marker.pose.position.y) / self._delta_t

        self.x = self._center_of_mass_marker.pose.position.x
        self.y = self._center_of_mass_marker.pose.position.y
        self.z = self._center_of_mass_marker.pose.position.z

        self._center_of_mass_marker = updated_center_of_mass
        self._prev_t = updated_center_of_mass.header.stamp

    def _calculate_capture_point(self, duration):
        """Calculate a future capture point pose using the inverted pendulum and center of mass.

        :param duration:
            the amount of seconds away from the current time the capture point should be calculated
        """
        rospy.logdebug('time diff: {td}'.format(td=str(self._delta_t)))

        try:
            world_transform = self._tf_buffer.lookup_transform('world', self._foot_link, rospy.Time())

            if self._delta_t != 0:
                rospy.loginfo('Calculating future center of mass for capture point pose.')
                new_center_of_mass = InvertedPendulum.numeric_solve_to_t(self.x - world_transform.transform.translation.x,
                                                                         self.y - world_transform.transform.translation.y,
                                                                         self.z - world_transform.transform.translation.z,
                                                                         self.vx, self.vy, duration)

                if new_center_of_mass['z'] <= 0:
                    rospy.logdebug_throttle(1, 'Cannot calculate capture point; center of mass < 0')

                capture_point_multiplier = sqrt(new_center_of_mass['z'] / self._gravity_constant)

                x_cp = new_center_of_mass['x'] + new_center_of_mass['vx'] * capture_point_multiplier
                y_cp = new_center_of_mass['y'] + new_center_of_mass['vy'] * capture_point_multiplier

                self._capture_point_marker.header.stamp = rospy.get_rostime()
                self._capture_point_marker.pose.position.x = x_cp + world_transform.transform.translation.x
                self._capture_point_marker.pose.position.y = y_cp + world_transform.transform.translation.y
                self._capture_point_marker.pose.position.z = 0

        except tf2_ros.TransformException as e:
            rospy.logdebug('Error in trying to lookup transform for capture point: {error}'.format(error=e))

        return {'time': self._center_of_mass_marker.header.stamp, 'x': self.x, 'y': self.y, 'z': self.z}

    def get_capture_point(self, capture_point_request_msg):
        """Service call function to return the capture point pose positions."""
        rospy.logdebug('Request capture point in {duration}'.format(duration=capture_point_request_msg.duration))

        duration = float(capture_point_request_msg.duration)
        self._calculate_capture_point(duration)

        return [True, 'Pose response from the capture point calculation.',
                str(self._capture_point_marker.pose.position.x),
                str(self._capture_point_marker.pose.position.y),
                str(self._capture_point_marker.pose.position.z)]
