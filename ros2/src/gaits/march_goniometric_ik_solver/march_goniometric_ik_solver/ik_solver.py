import numpy as np
import matplotlib.pyplot as plt
import time

import march_goniometric_ik_solver.quadrilateral_angle_solver as qas
import march_goniometric_ik_solver.triangle_angle_solver as tas

from march_utility.utilities.utility_functions import (
    get_lengths_robot_from_urdf_for_inverse_kinematics,
    get_limits_robot_from_urdf_for_inverse_kinematics,
)

# Get lengths from urdf:
lengths = get_lengths_robot_from_urdf_for_inverse_kinematics()
(LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, LENGTH_HIP_AA, LENGTH_HIP_BASE,) = (
    *lengths[0:3],
    lengths[-1],
)
LENGTH_LEG = LENGTH_UPPER_LEG + LENGTH_LOWER_LEG

# Get limits from urdf:
MAX_ANKLE_FLEXION = get_limits_robot_from_urdf_for_inverse_kinematics(
    "right_ankle"
).upper
MAX_HIP_ABDUCTION = get_limits_robot_from_urdf_for_inverse_kinematics(
    "right_hip_aa"
).upper
MAX_HIP_ADDUCTION = get_limits_robot_from_urdf_for_inverse_kinematics(
    "right_hip_aa"
).lower

# Constants:
LENGTH_FOOT = 0.10  # m
MIDDLE_POINT_HEIGHT = 0.15  # m

ANKLE_ZERO_ANGLE = np.pi / 2  # rad
KNEE_ZERO_ANGLE = np.pi  # rad
HIP_ZERO_ANGLE = np.pi  # rad

HIP_AA = 0.03  # rad

NUMBER_OF_JOINTS = 8


class Pose:
    def __init__(self, pose: list = [0] * NUMBER_OF_JOINTS):
        (
            self.fe_ankle1,
            self.aa_hip1,
            self.fe_hip1,
            self.fe_knee1,
            self.fe_ankle2,
            self.aa_hip2,
            self.fe_hip2,
            self.fe_knee2,
        ) = pose
        self.rot_foot1 = 0
        self.aa_hip1 = self.aa_hip2 = HIP_AA

    def reset_to_zero_pose(self):
        self.__init__()

    def rot(self, t):
        return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])

    def calculate_joint_positions(self, joint: str = "all"):
        """
        Calculates the joint positions for a given pose as a chain from rear toes (toes1) to front toes (toes2).
        If a positive angle represents a clockwise rotation, the angle variable is positive in the rot function.
        If a positive angle represents a non-clockwise rotation, the angle variable is negative in the rot function.
        The vectors (all np.array's) do always describe the translation when no rotation is applied.
        The rot_total matrix expands every step, since every joint location depends on all previous joint angles in the chain.
        """
        # create rotation matrix that we expand after every rotation:
        rot_total = self.rot(0)

        # toes1 is defined at [LENGTH_FOOT, 0]:
        pos_toes1 = np.array([LENGTH_FOOT, 0])

        # ankle1 = toes1 + translation_by_foot1:
        rot_foot = self.rot(-self.rot_foot1)
        rot_total = rot_total @ rot_foot
        pos_ankle1 = pos_toes1 + rot_total @ np.array([-LENGTH_FOOT, 0])

        # knee1 = ankle1 + translation_by_lower_leg1:
        rot_ankle1 = self.rot(-self.fe_ankle1)
        rot_total = rot_total @ rot_ankle1
        pos_knee1 = pos_ankle1 + rot_total @ np.array([0, LENGTH_LOWER_LEG])

        # hip = knee1 + translation_by_upper_leg1:
        rot_knee1 = self.rot(self.fe_knee1)
        rot_total = rot_total @ rot_knee1
        pos_hip = pos_knee1 + rot_total @ np.array([0, LENGTH_UPPER_LEG])

        # knee2 = hip + translation_by_upper_leg2:
        rot_hip = self.rot(self.fe_hip2 - self.fe_hip1)
        rot_total = rot_total @ rot_hip
        pos_knee2 = pos_hip + rot_total @ np.array([0, -LENGTH_UPPER_LEG])

        # ankle2 = knee2 + translation_by_lower_leg2:
        rot_knee2 = self.rot(-self.fe_knee2)
        rot_total = rot_total @ rot_knee2
        pos_ankle2 = pos_knee2 + rot_total @ np.array([0, -LENGTH_LOWER_LEG])

        # toe2 = ankle2 + translation_by_foot2:
        rot_ankle2 = self.rot(self.fe_ankle2)
        rot_total = rot_total @ rot_ankle2
        pos_toe2 = pos_ankle2 + rot_total @ np.array([LENGTH_FOOT, 0])

        # return all positions, or one specific if asked:
        if joint == "all":
            return (
                pos_toes1,
                pos_ankle1,
                pos_knee1,
                pos_hip,
                pos_knee2,
                pos_ankle2,
                pos_toe2,
            )
        else:
            return locals()[joint]

    def get_ankle_distance(self):
        pos_ankle1 = self.calculate_joint_positions("pos_ankle1")
        pos_ankle2 = self.calculate_joint_positions("pos_ankle2")
        return np.linalg.norm(pos_ankle1 - pos_ankle2)

    def get_pose(self):
        return (
            self.fe_ankle1,
            self.aa_hip1,
            self.fe_hip1,
            self.fe_knee1,
            self.fe_ankle2,
            self.aa_hip2,
            self.fe_hip2,
            self.fe_knee2,
        )

    def make_plot(self):
        """
        Makes a plot of the exo by first calculating the joint positions
        and then plotting them with lines.
        """

        positions = self.calculate_joint_positions()
        positions_x = [pos[0] for pos in positions]
        positions_y = [pos[1] for pos in positions]

        plt.figure(1)
        plt.plot(positions_x, positions_y)
        plt.gca().set_aspect("equal", adjustable="box")
        plt.show()

    def calculate_ground_pose_flexion(self, ankle_x: float):
        """
        Calculates and returns the flexion of the ankles and the hips when the
        ankle is moved to a certain x position, using pythagoras theorem.
        """

        return np.arcsin((ankle_x / 2) / LENGTH_LEG)

    def calculate_lifted_pose(self, pos_ankle2: np.array):
        """
        Calculate the pose after lifting the foot to the desired ankle postion.
        """

        # calculate angles using triangle between hip, knee2 and ankle2 and side distances:
        pos_hip = self.calculate_joint_positions("pos_hip")
        dist_hip_ankle = np.linalg.norm(pos_hip - pos_ankle2)
        sides = [LENGTH_LOWER_LEG, dist_hip_ankle, LENGTH_UPPER_LEG]
        angle_hip, angle_knee2, angle_ankle2 = tas.get_angles_from_sides(sides)

        # define new fe_hip2:
        point_below_hip = np.array([pos_ankle2[0] / 2, 0])
        hip_angle_vertical_ankle2 = qas.get_angle_between_points(
            [point_below_hip, pos_hip, pos_ankle2]
        )
        self.fe_hip2 = hip_angle_vertical_ankle2 + angle_hip

        # define new fe_knee2:
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2

        # define new fe_ankle2:
        toe2 = pos_ankle2 + np.array([LENGTH_FOOT, 0])
        ankle2_angle_toe2_hip = qas.get_angle_between_points(
            [toe2, pos_ankle2, pos_hip]
        )
        self.fe_ankle2 = ANKLE_ZERO_ANGLE - (ankle2_angle_toe2_hip - angle_ankle2)

    def reduce_swing_dorsi_flexion(self, max_flexion: float):
        """
        Calculate the pose after reducing the dorsiflexion using quadrilateral solver
        with quadrilateral between ankle2, knee2, hip, knee1
        """

        # get current state:
        (
            pos_ankle1,
            pos_knee1,
            pos_hip,
            pos_knee2,
            pos_ankle2,
        ) = self.calculate_joint_positions()[1:-1]

        # determine angle_ankle2 of quadrilateral:
        reduction = self.fe_ankle2 - max_flexion
        angle_ankle2 = (
            qas.get_angle_between_points([pos_knee1, pos_ankle2, pos_knee2]) - reduction
        )

        # determine other angles using angle_ankle2 and sides:
        dist_knee1_ankle2 = np.linalg.norm(pos_knee1 - pos_ankle2)
        sides = [
            LENGTH_UPPER_LEG,
            dist_knee1_ankle2,
            LENGTH_LOWER_LEG,
            LENGTH_UPPER_LEG,
        ]
        angle_knee1, angle_ankle2, angle_knee2, angle_hip = qas.solve_quadritlateral(
            sides, angle_ankle2
        )

        # define new fe_knee1:
        knee1_angle_ankle1_ankle2 = qas.get_angle_between_points(
            [pos_ankle1, pos_knee1, pos_ankle2]
        )
        self.fe_knee1 = angle_knee1 + knee1_angle_ankle1_ankle2 - KNEE_ZERO_ANGLE

        # get new hip location and determine point below it:
        pos_hip = self.calculate_joint_positions("pos_hip")
        point_below_hip = np.array([pos_hip[0], 0])

        # define new fe_hip1:
        if pos_hip[0] < pos_knee1[0]:
            self.fe_hip1 = qas.get_angle_between_points(
                [pos_knee1, pos_hip, point_below_hip]
            )
        else:
            self.fe_hip1 = -qas.get_angle_between_points(
                [pos_knee1, pos_hip, point_below_hip]
            )

        # define new fe_hip2, fe_knee2 and fe_ankle2:
        self.fe_hip2 = angle_hip + self.fe_hip1
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2
        self.fe_ankle2 -= reduction

    def straighten_leg(self):
        """
        Straighten stance leg by making a triangle between ankle1, hip and knee2
        and calculating new angles.
        """

        # get current state:
        (
            pos_toes1,
            pos_ankle1,
            pos_knee1,
            pos_hip,
            pos_knee2,
            pos_ankle2,
            pos_toe2,
        ) = self.calculate_joint_positions()

        # determine sides of triangle and calculate angles:
        dist_ankle1_knee2 = np.linalg.norm(pos_ankle1 - pos_knee2)
        sides = [LENGTH_UPPER_LEG, LENGTH_LEG, dist_ankle1_knee2]
        angle_ankle1, angle_knee2, angle_hip = tas.get_angles_from_sides(sides)

        # define new fe_ankle1 and fe_knee1:
        ankle1_angle_toes1_knee2 = qas.get_angle_between_points(
            [pos_knee2, pos_ankle1, pos_toes1]
        )
        self.fe_ankle1 = ANKLE_ZERO_ANGLE - (angle_ankle1 + ankle1_angle_toes1_knee2)
        self.fe_knee1 = 0

        # get new knee1 and hip location and determine point below it:
        pos_knee1 = self.calculate_joint_positions("pos_knee1")
        pos_hip = self.calculate_joint_positions("pos_hip")
        point_below_hip = np.array([pos_hip[0], 0])

        # define new fe_hip1:
        if pos_hip[0] < pos_knee1[0]:
            self.fe_hip1 = qas.get_angle_between_points(
                [pos_knee1, pos_hip, point_below_hip]
            )
        else:
            self.fe_hip1 = -qas.get_angle_between_points(
                [pos_knee1, pos_hip, point_below_hip]
            )

        # define new fe_hip2 and fe_knee2:
        self.fe_hip2 = angle_hip + self.fe_hip1
        knee2_angle_ankle1_ankle2 = qas.get_angle_between_points(
            [pos_ankle1, pos_knee2, pos_ankle2]
        )
        self.fe_knee2 = KNEE_ZERO_ANGLE - (angle_knee2 + knee2_angle_ankle1_ankle2)

    def reduce_stance_dorsi_flexion(self):
        # Save current angle at toes1 between ankle1 and hip:
        pos_ankle1 = self.calculate_joint_positions("pos_ankle1")
        pos_toes1 = self.calculate_joint_positions("pos_toes1")
        pos_hip = self.calculate_joint_positions("pos_hip")
        toes1_angle_ankle1_hip = qas.get_angle_between_points(
            [pos_ankle1, pos_toes1, pos_hip]
        )

        # Reduce dorsi flexion of stance leg:
        dis_toes1_hip = np.linalg.norm(pos_toes1 - pos_hip)
        lengths = [LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, LENGTH_FOOT, dis_toes1_hip]
        angle_knee1, angle_ankle1, angle_toes1, angle_hip = qas.solve_quadritlateral(
            lengths=lengths, angle_b=ANKLE_ZERO_ANGLE - MAX_ANKLE_FLEXION, convex=False
        )

        # Update pose:
        self.rot_foot1 = toes1_angle_ankle1_hip - angle_toes1
        self.fe_ankle1 = ANKLE_ZERO_ANGLE - angle_ankle1
        self.fe_knee1 = KNEE_ZERO_ANGLE - angle_knee1

        pos_knee1 = self.calculate_joint_positions("pos_knee1")
        point_below_hip = np.array([pos_hip[0], 0])
        self.fe_hip1 = np.sign(
            pos_knee1[0] - pos_hip[0]
        ) * qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])

    def perform_side_step(self, side_step: float):
        # determine side step distance per leg (assuming flat ground for now):
        dist_per_leg = side_step / 2 + LENGTH_HIP_AA

        # determine vertical distance of leg pose:
        dist_vertical = self.calculate_joint_positions("pos_hip")[1]

        n = (dist_per_leg * LENGTH_HIP_AA) / (LENGTH_HIP_AA ** 2 + dist_vertical ** 2)
        r_p1 = (dist_vertical ** 2 - dist_per_leg ** 2) / (
            dist_vertical ** 2 + LENGTH_HIP_AA ** 2
        )
        r_p2 = (dist_per_leg ** 2 * LENGTH_HIP_AA ** 2) / (
            (dist_vertical ** 2 + LENGTH_HIP_AA ** 2) ** 2
        )

        self.aa_hip1 = self.aa_hip2 = np.sign(side_step) * np.arccos(
            n + np.sqrt(r_p1 + r_p2)
        )

    def solve_mid_position(
        self,
        ankle_x: float,
        ankle_y: float,
        ankle_z: float,
        midpoint_fraction: float,
        subgait_id: str,
        plot: bool = False,
    ):
        """
        Solve inverse kinematics for the middle position. Assumes that the
        stance leg is straight. Takes the ankle_x and ankle_y position of the
        desired position and the midpoint_fraction at which a midpoint is desired.
        First calculates the midpoint position using current pose and fraction.
        Next, calculates the required hip and knee angles of
        the swing leg by making a triangle between the swing leg ankle, swing leg
        knee and the hip. Returns the calculated pose.
        """

        # Get swing distance in current pose and calculate ankle2 midpoint location:
        swing_distance = self.get_ankle_distance()
        midpoint_x = midpoint_fraction * (swing_distance + ankle_x) - swing_distance
        midpoint_y = ankle_y + MIDDLE_POINT_HEIGHT
        pos_ankle2 = np.array([midpoint_x, midpoint_y])

        # Reset pose to zero_pose and calculate distance between hip and ankle2 midpoint location:
        self.reset_to_zero_pose()
        pos_hip = self.calculate_joint_positions("pos_hip")
        dist_ankle2_hip = np.linalg.norm(pos_ankle2 - pos_hip)

        # Calculate hip and knee2 angles in triangle with ankle2:
        angle_hip, angle_knee2, angle_ankle2 = tas.get_angles_from_sides(
            [LENGTH_LOWER_LEG, dist_ankle2_hip, LENGTH_UPPER_LEG]
        )

        # fe_hip2 = angle_hip +- hip angle between ankle2 and knee1:
        pos_knee1 = self.calculate_joint_positions("pos_knee1")
        hip_angle_ankle2_knee1 = qas.get_angle_between_points(
            [pos_ankle2, pos_hip, pos_knee1]
        )
        self.fe_hip2 = angle_hip + np.sign(midpoint_x) * hip_angle_ankle2_knee1

        # update fe_knee2:
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2

        # lift toes as much as possible:
        self.fe_ankle2 = MAX_ANKLE_FLEXION

        # set hip_aa to average of current hip_aa and next hip_aa:
        if ankle_z != 0.0:
            next_pose = Pose()
            next_pose.solve_end_position(ankle_x, ankle_y, subgait_id, ankle_z)
            next_pose.perform_side_step(ankle_z)
            next_hip_aa = next_pose.aa_hip1
            self.aa_hip1 = self.aa_hip2 = (self.aa_hip1 + next_hip_aa) / 2

        if plot:
            self.make_plot()

        pose_list = self.get_pose()

        if subgait_id == "left_swing":
            half1 = pose_list[: len(pose_list) // 2]
            half2 = pose_list[len(pose_list) // 2 :]
            pose_list = half2 + half1

        return list(pose_list)

    def solve_end_position(
        self,
        ankle_x: float,
        ankle_y: float,
        ankle_z: float,
        subgait_id: str,
        max_ankle_flexion: float = MAX_ANKLE_FLEXION,
        plot: bool = False,
        timer: bool = False,
    ):
        """
        Solve inverse kinematics for a desired ankle location, assuming flat feet.
        Expects at least the ankle x-position and returns the calculated pose.
        """
        start = time.time()

        # make sure to start in zero_pose:
        self.reset_to_zero_pose()

        # calculate ground pose:
        ground_pose_flexion = self.calculate_ground_pose_flexion(ankle_x)
        self.fe_ankle1 = self.fe_hip2 = ground_pose_flexion
        self.fe_hip1 = self.fe_ankle2 = -ground_pose_flexion

        # calculate lifted pose if ankle_y > 0:
        if ankle_y > 0:
            pos_ankle = np.array([ankle_x, ankle_y])
            self.calculate_lifted_pose(pos_ankle)

            # reduce dorsi flexion of swing leg and straighten stance leg
            # if fe_ankle2 > max_flexion:
            if self.fe_ankle2 > max_ankle_flexion:

                # reduce dorsi flexion of swing leg:
                self.reduce_swing_dorsi_flexion(max_ankle_flexion)

                # straighten stance leg:
                self.straighten_leg()

        # reduce dorsi flexion of stance leg if fe_ankle1 > MAX_ANKLE_FLEXION:
        if self.fe_ankle1 > MAX_ANKLE_FLEXION:
            self.reduce_stance_dorsi_flexion()

        # add side step if ankle_z != 0:
        if ankle_z != 0.0:
            self.perform_side_step(ankle_z)

        if timer:
            end = time.time()
            print("Calculation time = ", end - start, " seconds")

        if plot:
            self.make_plot()

        pose_list = self.get_pose()

        if subgait_id == "left_swing":
            half1 = pose_list[: len(pose_list) // 2]
            half2 = pose_list[len(pose_list) // 2 :]
            pose_list = half2 + half1

        return list(pose_list)
