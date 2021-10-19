"""Module to calculate the capture point."""
from copy import deepcopy
from math import sqrt
from typing import Tuple

import numpy as np

from geometry_msgs.msg import Point, Pose
from march_data_collector.inverted_pendulum import InvertedPendulum
import rospy
import tf2_ros
from visualization_msgs.msg import Marker

from march_shared_msgs.srv import (
    CapturePointPose,
    CapturePointPoseResponse,
    CapturePointPoseRequest,
)

FRACTION_FALLING_TIME = 0.5


class CPCalculator:
    """Base class to calculate capture point for the exoskeleton.

    The capture point calculator is coupled to a static foot and swing foot. The static foot is used as the base of
    the inverted pendulum.
    """

    def __init__(
        self, tf_buffer: tf2_ros.Buffer, static_foot_link: str, swing_foot_link: str
    ):
        self._tf_buffer = tf_buffer
        self._static_foot_link = static_foot_link
        self.cp_service = rospy.Service(
            "/march/capture_point/" + swing_foot_link,
            CapturePointPose,
            self.get_capture_point,
        )

        self.cp_publisher = rospy.Publisher(
            "/march/cp_marker/" + swing_foot_link, Marker, queue_size=1
        )

        self._gravity_constant = 9.81
        self._prev_t = rospy.Time.now()
        self._delta_t = 0

        self.com_vx = 0
        self.com_vy = 0

        self._center_of_mass = Point()
        self._capture_point_marker = Marker()

        self._capture_point_duration = None

        self._capture_point_marker.header.frame_id = "world"
        self._capture_point_marker.type = self._capture_point_marker.SPHERE
        self._capture_point_marker.action = self._capture_point_marker.ADD
        self._capture_point_marker.pose.orientation.w = 1.0
        self._capture_point_marker.color.a = 1.0
        self._capture_point_marker.color.g = 1.0
        self._capture_point_marker.scale.x = 0.03
        self._capture_point_marker.scale.y = 0.03
        self._capture_point_marker.scale.z = 0.03

    @property
    def center_of_mass(self) -> Point:
        """Center of mass property getter."""
        return self._center_of_mass

    @center_of_mass.setter
    def center_of_mass(self, updated_center_of_mass: Marker):
        """Center of mass property setter."""
        if not isinstance(updated_center_of_mass, Marker):
            raise TypeError("Given center of mass is not of type: Marker")

        current_time = updated_center_of_mass.header.stamp
        self._delta_t = (current_time - self._prev_t).to_sec()

        if self._delta_t == 0:
            return

        self.com_vx = (
            updated_center_of_mass.pose.position.x - self._center_of_mass.x
        ) / self._delta_t
        self.com_vy = (
            updated_center_of_mass.pose.position.y - self._center_of_mass.y
        ) / self._delta_t

        self._center_of_mass = deepcopy(updated_center_of_mass.pose.position)
        self._prev_t = current_time

    def rotation_matrix_from_vectors(self, vec1, vec2):
        """ Find the rotation matrix that aligns vec1 to vec2
        :param vec1: A 3d "source" vector
        :param vec2: A 3d "destination" vector
        :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
        """
        a, b = (
            (vec1 / np.linalg.norm(vec1)).reshape(3),
            (vec2 / np.linalg.norm(vec2)).reshape(3),
        )
        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rotation_matrix

    def _calculate_capture_point(self, duration: float) -> Tuple[float, Pose]:
        """Calculate a future capture point pose using the inverted pendulum and center of mass.

        :param duration:
            the amount of seconds away from the current time the capture point should be calculated
        """
        try:
            static_foot_transform = self._tf_buffer.lookup_transform(
                "world", self._static_foot_link, rospy.Time()
            )
            static_foot_position = static_foot_transform.transform.translation
            falling_time = InvertedPendulum.calculate_falling_time(
                self._center_of_mass.x - static_foot_position.x,
                self._center_of_mass.y - static_foot_position.y,
                self._center_of_mass.z - static_foot_position.z,
                self.com_vx,
                self.com_vy,
            )

            capture_point_duration = min(duration, FRACTION_FALLING_TIME * falling_time)

            new_center_of_mass = InvertedPendulum.numeric_solve_to_t(
                self._center_of_mass.x - static_foot_position.x,
                self._center_of_mass.y - static_foot_position.y,
                self._center_of_mass.z - static_foot_position.z,
                self.com_vx,
                self.com_vy,
                capture_point_duration,
            )

            # make vectors for easy maths
            curr_com = np.array(
                [
                    self._center_of_mass.x - static_foot_position.x,
                    self._center_of_mass.y - static_foot_position.y,
                    self._center_of_mass.z - static_foot_position.z,
                ]
            )  # [x,y,z] relative to foot_pos
            new_com = np.array(
                [
                    new_center_of_mass["x"],
                    new_center_of_mass["y"],
                    new_center_of_mass["z"],
                ]
            )

            rotation_matrix = self.rotation_matrix_from_vectors(new_com, curr_com)

            if new_center_of_mass["z"] <= 0:
                rospy.loginfo_throttle(
                    1, "Cannot calculate capture point; z of new center of mass <= 0"
                )
                return -1, self._capture_point_marker.pose

            capture_point_multiplier = sqrt(
                new_center_of_mass["z"] / self._gravity_constant
            )

            x_cp = (
                new_center_of_mass["x"]
                + new_center_of_mass["vx"] * capture_point_multiplier
            )
            y_cp = (
                new_center_of_mass["y"]
                + new_center_of_mass["vy"] * capture_point_multiplier
            )

            # make vector for easy maths
            cp = np.array([x_cp, y_cp, 0])
            rotated_cp = rotation_matrix.dot(cp)

            self._capture_point_marker.header.stamp = rospy.get_rostime()
            # self._capture_point_marker.pose.position.x = x_cp + static_foot_position.x
            self._capture_point_marker.pose.position.x = (
                rotated_cp[0] + static_foot_position.x
            )
            # self._capture_point_marker.pose.position.y = y_cp + static_foot_position.y
            self._capture_point_marker.pose.position.y = (
                rotated_cp[1] + static_foot_position.y
            )
            # self._capture_point_marker.pose.position.z = 0
            self._capture_point_marker.pose.position.z = rotated_cp[2]

            self.cp_publisher.publish(self._capture_point_marker)
        except tf2_ros.TransformException as e:
            rospy.loginfo(
                "Error in trying to lookup transform for capture point: {error}".format(
                    error=e
                )
            )
            capture_point_duration = -1

        return capture_point_duration, self._capture_point_marker.pose

    def get_capture_point(
        self, capture_point_request_msg: CapturePointPoseRequest
    ) -> CapturePointPoseResponse:
        """Service call function to return the capture point pose positions."""
        duration = capture_point_request_msg.duration
        capture_point_duration, capture_point = self._calculate_capture_point(duration)
        if capture_point_duration < 0:
            return [False, 0.0, capture_point]

        return CapturePointPoseResponse(
            success=True, duration=capture_point_duration, capture_point=capture_point
        )
