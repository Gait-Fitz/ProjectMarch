"""Author: Jelmer de Wolde, MVII"""

import numpy as np
from typing import List, Tuple
import matplotlib.pyplot as plt

import march_goniometric_ik_solver.triangle_angle_solver as tas
import march_goniometric_ik_solver.quadrilateral_angle_solver as qas

from march_utility.utilities.utility_functions import (
    get_lengths_robot_from_urdf_for_inverse_kinematics,
    get_limits_robot_from_urdf_for_inverse_kinematics,
)

# Get lengths from urdf:
(
    LENGTH_UPPER_LEG,
    LENGTH_LOWER_LEG,
    LENGTH_HIP_AA,
    LENGTH_HIP_BASE,
) = get_lengths_robot_from_urdf_for_inverse_kinematics(
    length_names=["upper_leg", "lower_leg", "hip_aa_front", "hip_base"]
)
LENGTH_LEG = LENGTH_UPPER_LEG + LENGTH_LOWER_LEG
LENGTH_HIP = 2 * LENGTH_HIP_AA + LENGTH_HIP_BASE

# Get ankle limit from urdf:
limits = get_limits_robot_from_urdf_for_inverse_kinematics("right_ankle")
SOFT_LIMIT_BUFFER = np.deg2rad(3)
MAX_ANKLE_FLEXION = limits.upper - SOFT_LIMIT_BUFFER

# Constants:
LENGTH_FOOT = 0.10  # m

ANKLE_ZERO_ANGLE = np.pi / 2  # rad
KNEE_ZERO_ANGLE = np.pi  # rad
HIP_ZERO_ANGLE = np.pi  # rad

NUMBER_OF_JOINTS = 8
DEFAULT_HIP_X_FRACTION = 0.5
DEFAULT_KNEE_BEND = np.deg2rad(8)
MIDPOINT_HEIGHT = 0.1
ANKLE_TRAJECTORY_SAMPLES = 99


class Pose:
    """
    Used to solve inverse kinematics for a desired end_postion or mid_position of the foot.
    The class contains the joint_angles and the foot_rotation of the rear foot (in case of a toe-off)
    Solving can be done for the left or right foot, therefore this class uses the definition of 1 or 2
    for the joints, where 1 is the rear leg and 2 the front leg.
    Positive defined are: ankle dorsi-flexion, hip abduction, hip flexion, knee flexion.
    """

    def __init__(self, pose: List[float] = None) -> None:
        if pose is None:
            angle_ankle, angle_hip, angle_knee = self.leg_length_angles(
                self.max_leg_length
            )
            self.fe_ankle1 = self.fe_ankle2 = angle_ankle
            self.fe_hip1 = self.fe_hip2 = angle_hip
            self.fe_knee1 = self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee
            self.aa_hip1 = self.aa_hip2 = 0
        else:
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

    def reset_to_zero_pose(self) -> None:
        self.__init__()

    def swap_sides(self) -> None:
        self.fe_ankle1, self.fe_ankle2 = self.fe_ankle2, self.fe_ankle1
        self.aa_hip1, self.aa_hip2 = self.aa_hip2, self.aa_hip1
        self.fe_hip1, self.fe_hip2 = self.fe_hip2, self.fe_hip1
        self.fe_knee1, self.fe_knee2 = self.fe_knee2, self.fe_knee1

    @property
    def pose_right(self) -> List[float]:
        """
        Returns the pose as list with the right leg as front leg (leg2):
        """
        return [
            self.fe_ankle1,
            self.aa_hip1,
            self.fe_hip1,
            self.fe_knee1,
            self.fe_ankle2,
            self.aa_hip2,
            self.fe_hip2,
            self.fe_knee2,
        ]

    @property
    def pose_left(self) -> List[float]:
        """
        Returns the pose as list with the left leg as front leg (leg2):
        """
        return [
            self.fe_ankle2,
            self.aa_hip2,
            self.fe_hip2,
            self.fe_knee2,
            self.fe_ankle1,
            self.aa_hip1,
            self.fe_hip1,
            self.fe_knee1,
        ]

    def calculate_joint_positions(self, joint: str = "all") -> tuple:
        """
        Calculates the joint positions for a given pose as a chain from rear toes (toes1) to front toes (toes2).
        Can return all joint positions or a specific one, by choosing from:
        pos_toes1, pos_ankle1, pos_knee1, pos_hip, pos_knee2, pos_ankle2, pos_toes2

        If a positive angle represents a anti-clockwise rotation, the angle variable is positive in the rot function.
        If a positive angle represents a clockwise rotation, the angle variable is negative in the rot function.
        The vectors (all np.array's) do always describe the translation when no rotation is applied.
        The rot_total matrix expands every step, since every joint location depends on all previous joint angles in the chain.
        """
        # create rotation matrix that we expand after every rotation:
        rot_total = rot(0)

        # toes1 is defined at [LENGTH_FOOT, 0]:
        pos_toes1 = np.array([LENGTH_FOOT, 0])

        # ankle1 = toes1 + translation_by_foot1:
        rot_foot = rot(-self.rot_foot1)
        rot_total = rot_total @ rot_foot
        pos_ankle1 = pos_toes1 + rot_total @ np.array([-LENGTH_FOOT, 0])

        # knee1 = ankle1 + translation_by_lower_leg1:
        rot_ankle1 = rot(-self.fe_ankle1)
        rot_total = rot_total @ rot_ankle1
        pos_knee1 = pos_ankle1 + rot_total @ np.array([0, LENGTH_LOWER_LEG])

        # hip = knee1 + translation_by_upper_leg1:
        rot_knee1 = rot(self.fe_knee1)
        rot_total = rot_total @ rot_knee1
        pos_hip = pos_knee1 + rot_total @ np.array([0, LENGTH_UPPER_LEG])

        # knee2 = hip + translation_by_upper_leg2:
        rot_hip = rot(self.fe_hip2 - self.fe_hip1)
        rot_total = rot_total @ rot_hip
        pos_knee2 = pos_hip + rot_total @ np.array([0, -LENGTH_UPPER_LEG])

        # ankle2 = knee2 + translation_by_lower_leg2:
        rot_knee2 = rot(-self.fe_knee2)
        rot_total = rot_total @ rot_knee2
        pos_ankle2 = pos_knee2 + rot_total @ np.array([0, -LENGTH_LOWER_LEG])

        # toe2 = ankle2 + translation_by_foot2:
        rot_ankle2 = rot(self.fe_ankle2)
        rot_total = rot_total @ rot_ankle2
        pos_toes2 = pos_ankle2 + rot_total @ np.array([LENGTH_FOOT, 0])

        # return all positions, or one specific if asked:
        if joint == "all":
            return (
                pos_toes1,
                pos_ankle1,
                pos_knee1,
                pos_hip,
                pos_knee2,
                pos_ankle2,
                pos_toes2,
            )
        else:
            return locals()[joint]

    @property
    def pos_toes1(self) -> np.array:
        return self.calculate_joint_positions("pos_toes1")

    @property
    def pos_ankle1(self) -> np.array:
        return self.calculate_joint_positions("pos_ankle1")

    @property
    def pos_knee1(self) -> np.array:
        return self.calculate_joint_positions("pos_knee1")

    @property
    def pos_hip(self) -> np.array:
        return self.calculate_joint_positions("pos_hip")

    @property
    def pos_knee2(self) -> np.array:
        return self.calculate_joint_positions("pos_knee2")

    @property
    def pos_ankle2(self) -> np.array:
        return self.calculate_joint_positions("pos_ankle2")

    @property
    def pos_toes2(self) -> np.array:
        return self.calculate_joint_positions("pos_toes2")

    @property
    def point_below_hip(self) -> np.array:
        return np.array([self.pos_hip[0], 0])

    @property
    def hip_x(self) -> float:
        return self.ankle_x * self.hip_x_fraction

    @property
    def max_leg_length(self) -> float:
        """
        Returns the max net leg length (between ankle and hip) for the given knee_bend of the pose object.
        """
        pos_ankle = np.array([0, 0])
        pos_knee = pos_ankle + np.array([0, LENGTH_LOWER_LEG])

        try:
            knee_angle = self.knee_bend
        except AttributeError:
            knee_angle = DEFAULT_KNEE_BEND

        pos_hip = pos_knee + rot(knee_angle) @ np.array([0, LENGTH_UPPER_LEG])
        return np.linalg.norm(pos_hip - pos_ankle)

    @property
    def ankle_limit_toes_knee_distance(self) -> float:
        """
        Returns the distance between knee and toes when the ankle is in max dorsi flexion.
        """
        pose = Pose()
        pose.fe_ankle1 = MAX_ANKLE_FLEXION
        return np.linalg.norm(pose.pos_toes1 - pose.pos_knee1)

    @property
    def max_swing_dorsi_flexion_reduction(self) -> float:
        """
        Returns the maximum possible dorsiflexion reduction for the swing ankle in the current pose.
        """
        angle_ankle2, angle_knee2, angle_hip, angle_knee1 = qas.get_angles(
            [self.pos_ankle2, self.pos_knee2, self.pos_hip, self.pos_knee1]
        )

        hip_above_rear_ankle_pose = Pose()
        pos_hip = np.array([0, self.max_leg_length])
        pos_ankle1 = np.array([0, 0])
        pos_ankle2 = self.pos_ankle2
        hip_above_rear_ankle_pose.solve_leg(pos_hip, pos_ankle1, "rear")
        hip_above_rear_ankle_pose.solve_leg(pos_hip, pos_ankle2, "front")

        max_reduction = self.fe_ankle2 - hip_above_rear_ankle_pose.fe_ankle2
        print("max_reduction = ", max_reduction)

        return min(
            angle_ankle2,
            KNEE_ZERO_ANGLE - angle_knee2 - DEFAULT_KNEE_BEND,
            angle_hip,
            KNEE_ZERO_ANGLE - angle_knee1 - DEFAULT_KNEE_BEND,
            max_reduction,
        )

    def leg_length_angles(self, leg_length: float) -> Tuple[float]:
        """
        Returns the required angles in the triangle between ankle, hip and knee
        to meet the net leg_length (from ankle to hip) given as input argument.
        """
        if leg_length < LENGTH_LEG:
            sides = [LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, leg_length]
            angle_ankle, angled_hip, angle_knee = tas.get_angles_from_sides(sides)
            return angle_ankle, angled_hip, angle_knee
        else:
            return 0.0, 0.0, np.pi

    def solve_leg(self, pos_hip: np.array, pos_ankle: np.array, leg: str) -> None:
        """
        Set the required joint angles for the given leg to reach given pos_ankle location
        with hip at given pos_hip location. Expects a 2D numpy array for both pos_hip and
        pos_ankle containing the x and y location. The given leg can be 'rear' or 'front'.
        """
        dist_ankle_hip = np.linalg.norm(pos_hip - pos_ankle)
        angle_ankle, angle_hip, angle_knee = self.leg_length_angles(dist_ankle_hip)

        base_angle = np.arcsin(abs(pos_ankle[0] - pos_hip[0]) / dist_ankle_hip)

        # Note: np.signs are there for when hip is behind ankle (can occur in mid position)
        if leg == "rear":
            self.fe_ankle1 = (
                np.sign(pos_hip[0] - pos_ankle[0]) * base_angle + angle_ankle
            )
            self.fe_hip1 = np.sign(pos_ankle[0] - pos_hip[0]) * base_angle + angle_hip
            self.fe_knee1 = KNEE_ZERO_ANGLE - angle_knee

        elif leg == "front":
            self.fe_ankle2 = (
                np.sign(pos_hip[0] - pos_ankle[0]) * base_angle + angle_ankle
            )
            self.fe_hip2 = np.sign(pos_ankle[0] - pos_hip[0]) * base_angle + angle_hip

            self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee

        else:
            raise ValueError("Expected leg to be 'rear' or 'front'.")

    def solve_foot_orientation(self, foot_rotation):
        desired_fe_ankle2 = self.fe_ankle2 - foot_rotation
        desired_exceeding = desired_fe_ankle2 - MAX_ANKLE_FLEXION

        if desired_exceeding < self.max_swing_dorsi_flexion_reduction:
            self.fe_ankle2 -= foot_rotation
        else:
            self.fe_ankle2 = MAX_ANKLE_FLEXION + self.max_swing_dorsi_flexion_reduction

        print("desired_exceeding =                 ", desired_exceeding)
        print(
            "max_swing_dorsi_flexion_reduction = ",
            self.max_swing_dorsi_flexion_reduction,
        )
        print("\n")

    def reduce_swing_dorsi_flexion(self) -> None:
        """
        Calculates the pose after reducing the dorsiflexion using quadrilateral solver
        with quadrilateral between ankle2, knee2, hip, knee1.
        """

        # Determine required reduction:
        reduction = self.fe_ankle2 - MAX_ANKLE_FLEXION

        # Store current angle of ankle1 between ankle2 and hip:
        angle_ankle1_before = qas.get_angle_between_points(
            [self.pos_ankle2, self.pos_ankle1, self.pos_hip]
        )

        # Define desired angle_ankle2 and determine other angles in quadrilateral:
        angle_ankle2 = (
            qas.get_angle_between_points(
                [self.pos_ankle1, self.pos_ankle2, self.pos_knee2]
            )
            - reduction
        )
        dist_ankle1_ankle2 = np.linalg.norm(self.pos_ankle1 - self.pos_ankle2)
        sides = [
            self.max_leg_length,
            dist_ankle1_ankle2,
            LENGTH_LOWER_LEG,
            LENGTH_UPPER_LEG,
        ]
        angle_ankle1, angle_ankle2, angle_knee2, angle_hip = qas.solve_quadritlateral(
            sides, angle_ankle2
        )

        # Define new fe_ankle1:
        self.fe_ankle1 = self.fe_ankle1 - (angle_ankle1 - angle_ankle1_before)

        # Define other joint angles:
        self.fe_hip1 = np.sign(
            self.pos_knee1[0] - self.pos_hip[0]
        ) * qas.get_angle_between_points(
            [self.pos_knee1, self.pos_hip, self.point_below_hip]
        )
        self.fe_hip2 = (
            angle_hip
            - qas.get_angle_between_points(
                [self.pos_ankle1, self.pos_hip, self.pos_knee1]
            )
            + self.fe_hip1
        )
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2
        self.fe_ankle2 -= reduction

    def keep_hip_above_rear_ankle(self) -> None:
        """
        Calculates the pose required to keep the hip above the rear ankle
        while reaching at least the goal location for the toes.
        """

        # Store desired toes location and reset pose:
        pos_toes2 = np.array([self.ankle_x + LENGTH_FOOT, self.ankle_y])
        self.reset_to_zero_pose()

        # Calculate angles in triangle between hip, toes2 and knee2:
        dist_hip_toes2 = np.linalg.norm(self.pos_hip - pos_toes2)
        dist_toes2_knee2 = self.ankle_limit_toes_knee_distance
        angle_hip, angle_toes2, angle_knee2 = tas.get_angles_from_sides(
            [dist_toes2_knee2, LENGTH_UPPER_LEG, dist_hip_toes2]
        )

        # Calculate outer angles of hip and knee:
        angle_hip_out = qas.get_angle_between_points(
            [self.point_below_hip, self.pos_hip, pos_toes2]
        )
        angle_knee2_out = tas.get_angle_from_sides(
            LENGTH_FOOT, np.array([LENGTH_LOWER_LEG, dist_toes2_knee2])
        )

        # Define required joint angles to reach toes goal:
        self.fe_hip2 = angle_hip + angle_hip_out
        self.fe_knee2 = KNEE_ZERO_ANGLE - (angle_knee2 - angle_knee2_out)
        self.fe_ankle2 = MAX_ANKLE_FLEXION

    def reduce_stance_dorsi_flexion(self) -> None:
        """
        Calculates the pose after reducing the dorsiflexion using quadrilateral solver
        with quadrilateral between toes1, ankle1, knee1, and hip.
        """

        # Save current angle at toes1 between ankle1 and hip:
        toes1_angle_ankle1_hip = qas.get_angle_between_points(
            [self.pos_ankle1, self.pos_toes1, self.pos_hip]
        )

        # Reduce dorsi flexion of stance leg:
        dis_toes1_hip = np.linalg.norm(self.pos_toes1 - self.pos_hip)
        lengths = [LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, LENGTH_FOOT, dis_toes1_hip]
        angle_knee1, angle_ankle1, angle_toes1, angle_hip = qas.solve_quadritlateral(
            lengths=lengths, angle_b=ANKLE_ZERO_ANGLE - MAX_ANKLE_FLEXION, convex=False
        )

        # Update pose:
        self.rot_foot1 = toes1_angle_ankle1_hip - angle_toes1
        self.fe_ankle1 = ANKLE_ZERO_ANGLE - angle_ankle1
        self.fe_knee1 = KNEE_ZERO_ANGLE - angle_knee1
        self.fe_hip1 = np.sign(
            self.pos_knee1[0] - self.pos_hip[0]
        ) * qas.get_angle_between_points(
            [self.pos_knee1, self.pos_hip, self.point_below_hip]
        )

    def perform_side_step(self, y: float, z: float):
        """
        A method that calculates the required hip adduction/abduction for a given feet distance z,
        without changing the vertical feet distance y. Requires very complex equations.
        See the README.MD of this package for more information.
        """

        # Get y position of hip and toes:
        y_hip = self.pos_hip[1]
        y_toes1 = self.pos_toes1[1]
        y_toes2 = self.pos_toes2[1]

        # Determine lengths:
        length_leg_short, length_leg_long = sorted([y_hip - y_toes1, y_hip - y_toes2])
        length_short, length_long = (
            np.sqrt(length_leg_short ** 2 + LENGTH_HIP_AA ** 2),
            np.sqrt(length_leg_long ** 2 + LENGTH_HIP_AA ** 2),
        )

        # Calculate theta_long and theta_short:
        # Note that flake8-expression-complexity check is disabled since this is a very complex calculation.
        theta_long = 2 * np.arctan(  # noqa: ECE001
            (
                -2 * length_long * LENGTH_HIP_BASE
                + 2 * length_long * z
                - np.sqrt(
                    -(length_long ** 4)
                    + 2 * length_long ** 2 * length_short ** 2
                    + 2 * length_long ** 2 * LENGTH_HIP_BASE ** 2
                    - 4 * length_long ** 2 * LENGTH_HIP_BASE * z
                    + 2 * length_long ** 2 * y ** 2
                    + 2 * length_long ** 2 * z ** 2
                    - length_short ** 4
                    + 2 * length_short ** 2 * LENGTH_HIP_BASE ** 2
                    - 4 * length_short ** 2 * LENGTH_HIP_BASE * z
                    + 2 * length_short ** 2 * y ** 2
                    + 2 * length_short ** 2 * z ** 2
                    - LENGTH_HIP_BASE ** 4
                    + 4 * LENGTH_HIP_BASE ** 3 * z
                    - 2 * LENGTH_HIP_BASE ** 2 * y ** 2
                    - 6 * LENGTH_HIP_BASE ** 2 * z ** 2
                    + 4 * LENGTH_HIP_BASE * y ** 2 * z
                    + 4 * LENGTH_HIP_BASE * z ** 3
                    - y ** 4
                    - 2 * y ** 2 * z ** 2
                    - z ** 4
                )
            )
            / (
                length_long ** 2
                + 2 * length_long * y
                - length_short ** 2
                + LENGTH_HIP_BASE ** 2
                - 2 * LENGTH_HIP_BASE * z
                + y ** 2
                + z ** 2
            )
        )

        theta_short = np.arccos((length_long * np.cos(theta_long) - y) / length_short)

        # Determine hip_aa for both hips:
        angle_hip_long = tas.get_angle_from_sides(
            length_leg_long, np.array([length_long, LENGTH_HIP_AA])
        )
        angle_hip_short = tas.get_angle_from_sides(
            length_leg_short, np.array([length_short, LENGTH_HIP_AA])
        )

        hip_aa_long = theta_long + angle_hip_long - HIP_ZERO_ANGLE / 2
        hip_aa_short = theta_short + angle_hip_short - HIP_ZERO_ANGLE / 2

        if y_toes1 > y_toes2:
            self.aa_hip1 = hip_aa_short
            self.aa_hip2 = hip_aa_long
        else:
            self.aa_hip1 = hip_aa_long
            self.aa_hip2 = hip_aa_short

    def create_ankle_trajectory(self, next_pose: "Pose"):
        """
        Create a ankle trajectory from current pose (self) to given next_pose.
        """
        # Get ankle positions via the static toes:
        ankle_start = self.pos_ankle1
        toes_static = self.pos_toes2
        ankle_end = toes_static + (next_pose.pos_ankle2 - next_pose.pos_toes1)

        # Calculate step size and height:
        step_size = ankle_end[0] - ankle_start[0]
        step_height = ankle_end[1] - ankle_start[1]

        # Determine the parabola function:
        if step_size != 0:
            c = MIDPOINT_HEIGHT
            a = -4 * (c / step_size ** 2)
            x = np.linspace(0, 1, ANKLE_TRAJECTORY_SAMPLES) * step_size - step_size / 2
            y_parabola = a * x ** 2 + c
        else:
            y_parabola = 0

        # Define trajectory:
        x = np.linspace(0, 1, ANKLE_TRAJECTORY_SAMPLES) * step_size
        y = np.linspace(0, 1, ANKLE_TRAJECTORY_SAMPLES) * step_height + y_parabola

        return x, y

    def solve_mid_position(
        self,
        next_pose: "Pose",
        frac: float,
        subgait_id: str,
    ) -> List[float]:
        """
        Solve inverse kinematics for a middle position at a given frac between current pose (self) and given next_pose.
        Returns the calculated pose as a list.
        """
        # Store current hip_aa to calculate hip_aa of midpoint later:
        current_hip_aa_1, current_hip_aa_2 = self.aa_hip1, self.aa_hip2

        # Translate current hip and ankle location to new reference frame:
        hip_current = self.pos_hip - self.pos_ankle2
        ankle_current = self.pos_ankle1 - self.pos_ankle2

        # Create ankle_trajectory and get ankle_mid location of given frac:
        ankle_trajectory = np.array(self.create_ankle_trajectory(next_pose))
        index = round(np.shape(ankle_trajectory)[1] * frac)
        ankle_mid = ankle_current + ankle_trajectory[:, index]

        # Define hip_mid_x as the given fraction between current hip location and the hip location in next pose:
        hip_mid_x = (1 - frac) * hip_current[0] + frac * next_pose.pos_hip[0]

        # Define hip_mid_y as the minimum of the heights both legs can reach:
        max_height_swing_leg = ankle_mid[1] + np.sqrt(
            self.max_leg_length ** 2 - (ankle_mid[0] - hip_mid_x) ** 2
        )
        max_height_stance_leg = np.sqrt(self.max_leg_length ** 2 - (hip_mid_x) ** 2)
        hip_mid_y = min(max_height_swing_leg, max_height_stance_leg)

        hip_mid = np.array([hip_mid_x, hip_mid_y])

        # Solve the mid pose with the define hip and ankle positions:
        self.reset_to_zero_pose()
        self.solve_leg(hip_mid, self.pos_ankle1, "rear")
        self.solve_leg(hip_mid, ankle_mid, "front")
        if self.fe_ankle1 > MAX_ANKLE_FLEXION:
            self.reduce_stance_dorsi_flexion()

        # Lift toes of swing leg as much as possible:
        self.fe_ankle2 = MAX_ANKLE_FLEXION

        # Set hip_aa to average of start and end pose:
        self.aa_hip1 = current_hip_aa_1 * (1 - frac) + next_pose.aa_hip1 * frac
        self.aa_hip2 = current_hip_aa_2 * (1 - frac) + next_pose.aa_hip2 * frac

        # Return pose as list:
        return self.pose_left if (subgait_id == "left_swing") else self.pose_right

    def solve_end_position(
        self,
        ankle_x: float,
        ankle_y: float,
        ankle_z: float,
        subgait_id: str,
        foot_rotation: float = 0.0,
        hip_x_fraction: float = DEFAULT_HIP_X_FRACTION,
        default_knee_bend: float = DEFAULT_KNEE_BEND,
        reduce_df_front: bool = True,
        reduce_df_rear: bool = True,
    ) -> List[float]:
        """
        Solve inverse kinematics for a desired ankle location.
        Returns the calculated pose as a list.
        """
        # Reset to zero pose:
        self.reset_to_zero_pose()

        # Set parameters:
        self.ankle_x = ankle_x
        self.ankle_y = ankle_y
        self.hip_x_fraction = hip_x_fraction
        self.knee_bend = default_knee_bend

        # Determine hip y-location:
        if ankle_y > 0:
            if hip_x_fraction >= 0.5:
                hip_y = np.sqrt(self.max_leg_length ** 2 - (self.hip_x) ** 2)
            else:
                hip_y = np.sqrt(self.max_leg_length ** 2 - (ankle_x - self.hip_x) ** 2)
        else:
            hip_y = min(
                ankle_y
                + np.sqrt(self.max_leg_length ** 2 - (ankle_x - self.hip_x) ** 2),
                np.sqrt(self.max_leg_length ** 2 - (self.hip_x) ** 2),
            )

        # Define hip and ankle locations:
        pos_hip = np.array([self.hip_x, hip_y])
        pos_ankle1 = np.array([0, 0])
        pos_ankle2 = np.array([ankle_x, ankle_y])

        # Solve legs without constraints:
        self.solve_leg(pos_hip, pos_ankle1, "rear")
        self.solve_leg(pos_hip, pos_ankle2, "front")

        # Apply foot rotation if possible:
        # self.solve_foot_orientation(foot_rotation)

        # Reduce dorsi flexion to meet constraints:
        if reduce_df_front and self.fe_ankle2 > MAX_ANKLE_FLEXION:
            self.reduce_swing_dorsi_flexion()
            if self.pos_hip[0] < self.pos_ankle1[0]:
                self.keep_hip_above_rear_ankle()

        if reduce_df_rear and self.fe_ankle1 > MAX_ANKLE_FLEXION:
            self.reduce_stance_dorsi_flexion()

        # Apply side_step, hard_coded to default feet distance for now:
        self.perform_side_step(ankle_y, abs(ankle_z))

        # return pose as list:
        return self.pose_left if (subgait_id == "left_swing") else self.pose_right


# Static methods:
def rot(t: float) -> np.array:
    """
    Returns the 2D rotation matrix R to rotate a vector with rotation t (in rad), so that:
    ⎡x'⎤ ⎽ ⎡cos(t) -sin(t)⎤⎡x⎤
    ⎣y'⎦ ⎺ ⎣sin(t)  cos(t)⎦⎣y⎦
    A positive value of t results in a anti-clockwise rotation around the origin.
    """
    return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])


def make_plot(pose: Pose):
    """
    Makes a plot of the exo by first calculating the joint positions and then plotting them.
    This method is only used for debugging reasons.
    """

    positions = pose.calculate_joint_positions()
    positions_x = [pos[0] for pos in positions]
    positions_y = [pos[1] for pos in positions]

    plt.figure(1)
    plt.plot(positions_x, positions_y, ".-")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()
