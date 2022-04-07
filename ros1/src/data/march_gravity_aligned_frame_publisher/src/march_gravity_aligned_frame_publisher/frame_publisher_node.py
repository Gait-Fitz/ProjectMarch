"""Author: Jelmer de Wolde, MVII"""

import rospy
import tf

# Define the distance from frame 'foot_left' or 'foot_right' at which the foot rotates:
FOOT_LENGTH = 0.2
Z_ROTATION = [0.0, 0.0, 0.0, 1.0]
RATE = 10.0


def main():
    """
    A node that publishes two frames around toes for both feet.
    The first frame is a transformation from the 'foot_left'/'foot_right' to the toes, rotated 180 degrees around the z-axis.
    The second frame is te first described frame, but with the z-axis aligned with the gravitation force.
    """
    rospy.init_node("frame_aligner")

    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(RATE)
    last_published_timestamp = rospy.Time(0)

    while not rospy.is_shutdown():
        try:
            (_, rot_left) = listener.lookupTransform("/foot_left", "/world", rospy.Time(0))
            (_, rot_right) = listener.lookupTransform("/foot_right", "/world", rospy.Time(0))
            (_, rot_hip) = listener.lookupTransform("/hip_base", "/world", rospy.Time(0))
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        current_timestamp = rospy.Time.now()

        if last_published_timestamp < current_timestamp:
            broadcaster.sendTransform(
                (-FOOT_LENGTH, 0.0, -0.01),
                Z_ROTATION,
                current_timestamp,
                "toes_left",
                "foot_left",
            )

            broadcaster.sendTransform(
                (0.0, 0.0, 0.0),
                rot_left,
                current_timestamp,
                "toes_left_aligned",
                "toes_left",
            )

            broadcaster.sendTransform(
                (-FOOT_LENGTH, 0.0, -0.01),
                Z_ROTATION,
                current_timestamp,
                "toes_right",
                "foot_right",
            )

            broadcaster.sendTransform(
                (0.0, 0.0, 0.0),
                rot_right,
                current_timestamp,
                "toes_right_aligned",
                "toes_right",
            )

            broadcaster.sendTransform(
                (0.0, 0.0, 0.0),
                rot_hip,
                current_timestamp,
                "hip_base_aligned",
                "hip_base",
            )

            last_published_timestamp = current_timestamp

        rate.sleep()
