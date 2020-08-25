from math import sqrt

from geometry_msgs.msg import Point
from march_data_collector.inverted_pendulum import InvertedPendulum
import rospy
from visualization_msgs.msg import Marker

from march_shared_resources.srv import CapturePointPose


class CPCalculator(object):

    def __init__(self):
        """Base class to calculate capture point for the exoskeleton."""
        self.cp_service = rospy.Service('/march/capture_point', CapturePointPose, self.get_capture_point)

        self.cp_publisher = rospy.Publisher('/march/cp_marker', Marker, queue_size=1)

        self._gravity_constant = 9.81
        self._prev_t = rospy.Time.now()
        self._delta_t = 0

        self.x = 0
        self.y = 0
        self.z = 0
        self.vx = 0
        self.vy = 0

        self._center_of_mass = Point()
        self._capture_point_marker = Marker()

        self._capture_point_duration = None

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
    def center_of_mass(self):
        """Center of mass property getter."""
        return self._center_of_mass

    @center_of_mass.setter
    def center_of_mass(self, updated_center_of_mass):
        """Center of mass property setter."""
        if not isinstance(updated_center_of_mass, Marker):
            raise TypeError('Given center of mass is not of type: Marker')

        current_time = updated_center_of_mass.header.stamp
        self._delta_t = (current_time - self._prev_t).to_sec()

        if self._delta_t == 0:
            return

        self.vx = (updated_center_of_mass.pose.position.x - self._center_of_mass.x) / self._delta_t
        self.vy = (updated_center_of_mass.pose.position.y - self._center_of_mass.y) / self._delta_t

        self._center_of_mass = updated_center_of_mass.pose.position
        self._prev_t = current_time

    def _calculate_capture_point(self, duration):
        """Calculate a future capture point pose using the inverted pendulum and center of mass.

        :param duration:
            the amount of seconds away from the current time the capture point should be calculated
        """
        falling_time = InvertedPendulum.calculate_falling_time(
            self._center_of_mass.x,
            self._center_of_mass.y,
            self._center_of_mass.z,
            self.vx, self.vy)

        capture_point_duration = min(duration, 0.5 * falling_time)

        new_center_of_mass = InvertedPendulum.numeric_solve_to_t(
            self._center_of_mass.x,
            self._center_of_mass.y,
            self._center_of_mass.z,
            self.vx, self.vy, capture_point_duration)

        if new_center_of_mass['z'] <= 0:
            rospy.logdebug_throttle(1, 'Cannot calculate capture point; z of new center of mass <= 0')

        capture_point_multiplier = sqrt(new_center_of_mass['z'] / self._gravity_constant)

        x_cp = new_center_of_mass['x'] + new_center_of_mass['vx'] * capture_point_multiplier
        y_cp = new_center_of_mass['y'] + new_center_of_mass['vy'] * capture_point_multiplier

        self._capture_point_marker.header.stamp = rospy.get_rostime()
        self._capture_point_marker.pose.position.x = x_cp
        self._capture_point_marker.pose.position.y = y_cp
        self._capture_point_marker.pose.position.z = 0

        self.cp_publisher.publish(self._capture_point_marker)
        return capture_point_duration

    def get_capture_point(self, capture_point_request_msg):
        """Service call function to return the capture point pose positions."""
        rospy.logdebug('Request capture point in {duration}'.format(duration=capture_point_request_msg.duration))

        duration = capture_point_request_msg.duration
        capture_point_duration = self._calculate_capture_point(duration)

        return [True, capture_point_duration, self._capture_point_marker.pose]
