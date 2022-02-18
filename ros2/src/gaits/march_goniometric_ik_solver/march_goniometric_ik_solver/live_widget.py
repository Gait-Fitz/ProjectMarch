"""Author: Jelmer de Wolde, MVII"""

import pkg_resources
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from march_goniometric_ik_solver.ik_solver import Pose, LENGTH_FOOT
from copy import deepcopy

# Layout problems occurred with older versions of matplotlib (for example 3.1.2).
# Tested and working on 3.5.1 (latest version at the moment this tool was made).
# If newer versions release, test the layout and adjust requirement if successfull:
pkg_resources.require("matplotlib==3.5.1")


class LiveWidget:
    """
    A widget to easily check the solutions of the IK solver for a given x,y location of the ankle.
    This widget has been made for debugging purposes, to evaluate poses the IK solver provides as
    solution for a given goal location. This widget can be executed by sourcing ROS2, March ROS2
    and running this script with python: sfox && sros2 && python3 live_widget.py
    """

    def __init__(self) -> None:
        # Define default values:
        self.default_hip_fraction = 0.5
        self.default_knee_bend = np.deg2rad(8)
        self.reduce_df_rear = True
        self.reduce_df_front = True
        self.fig, self.ax = plt.subplots()
        self.playing = False

        # Create pose object for current pose, mid_pose and next pose:
        self.current_pose = {
            "name": "current_pose",
            "pose": Pose(),
            "color": "red",
            "start_cell": 1,
        }
        self.mid_pose = {
            "name": "mid_pose",
            "pose": Pose(),
            "color": "orange",
            "start_cell": 10,
        }
        self.next_pose = {
            "name": "next_pose",
            "pose": Pose(),
            "color": "green",
            "start_cell": 19,
        }
        self.poses = [self.current_pose, self.mid_pose, self.next_pose]
        (self.trajectory,) = plt.plot(0, 0, ".-", color="blue")

        for pose in self.poses:
            positions = pose["pose"].calculate_joint_positions()

            # Translate current pose to have toes2 and in (0,0):
            if pose["name"] == "current_pose":
                positions = [pos - positions[-1] for pos in positions]
            else:
                positions = [pos - np.array([LENGTH_FOOT, 0]) for pos in positions]

            # Plot the pose:
            positions_x = [pos[0] for pos in positions]
            positions_y = [pos[1] for pos in positions]
            (pose["plot"],) = plt.plot(
                positions_x, positions_y, ".-", color=pose["color"]
            )

            # Plot ankle and toes goal locations for next_pose:
            if pose["name"] == "next_pose":
                (pose["goal_ankle"],) = plt.plot(-LENGTH_FOOT, 0.0, "x", color="black")
                (pose["goal_toes"],) = plt.plot(0.0, 0.0, "x", color="black")

            # Create table celltext for pose:
            joints = [
                "ankle1",
                "hip1_aa",
                "hip1_fe",
                "knee1",
                "ankle2",
                "hip2_aa",
                "hip2_fe",
                "knee2",
            ]
            pose_rad = pose["pose"].pose_left
            pose_deg = [np.rad2deg(angle) for angle in pose_rad]
            pose["celltext"] = np.column_stack(
                (joints, np.round(pose_rad, 2), np.round(pose_deg, 2))
            )

        # Create table:
        collabels = ("joint", "radians", "degrees")
        celltext = np.concatenate(
            [
                self.current_pose["celltext"],
                np.array([["", "", ""]]),
                self.mid_pose["celltext"],
                np.array([["", "", ""]]),
                self.next_pose["celltext"],
            ]
        )
        self.table = plt.table(cellText=celltext, colLabels=collabels, loc="right")
        self.table.auto_set_column_width((0, 1, 2))
        self.table.scale(1, 0.7)

        # Adjust the main plot to make room for the table, sliders and buttons:
        plt.subplots_adjust(left=0.1, bottom=0.2, right=2.5)
        plt.axis("equal")
        plt.xlim(-0.6, 0.6)
        plt.ylim(-0.3, 0.9)
        plt.tight_layout()
        plt.grid()

        # Make a horizontal slider to control the x location of the ankle:
        slider_handle = {"size": "5"}
        ax_ankle_x = plt.axes([0.09, 0.01, 0.29, 0.02])
        self.x_slider_current = Slider(
            ax=ax_ankle_x,
            label="",
            valmin=-0.6,
            valmax=0.0,
            valinit=0.0,
            color=self.current_pose["color"],
            initcolor="black",
            handle_style=slider_handle,
        )
        self.x_slider_current.valtext.set_visible(False)

        ax_ankle_x = plt.axes([0.385, 0.01, 0.29, 0.02])
        self.x_slider_next = Slider(
            ax=ax_ankle_x,
            label="",
            valmin=0.0,
            valmax=0.6,
            valinit=0.0,
            color=self.next_pose["color"],
            initcolor="black",
            handle_style=slider_handle,
        )
        self.x_slider_next.valtext.set_visible(False)

        # Make a vertical slider to control the y location of the ankle:
        ax_ankle_y = plt.axes([0.01, 0.08, 0.02, 0.885])
        self.y_slider_current = Slider(
            ax=ax_ankle_y,
            label="",
            valmin=-0.3,
            valmax=0.3,
            valinit=0.0,
            orientation="vertical",
            color=self.current_pose["color"],
        )

        ax_ankle_y = plt.axes([0.03, 0.08, 0.02, 0.885])
        self.y_slider_next = Slider(
            ax=ax_ankle_y,
            label="",
            valmin=-0.3,
            valmax=0.3,
            valinit=0.0,
            orientation="vertical",
            color=self.next_pose["color"],
        )

        # Make a slider to control hip_fraction:
        ax_horizontal = plt.axes([0.74, 0.93, 0.2, 0.02])
        self.hip_slider = Slider(
            ax=ax_horizontal,
            label="hip",
            valmin=0.0,
            valmax=1.0,
            valinit=0.5,
        )

        # Make a slider to control knee_bend:
        ax_horizontal = plt.axes([0.74, 0.96, 0.2, 0.02])
        self.knee_slider = Slider(
            ax=ax_horizontal,
            label="knee",
            valmin=0,
            valmax=10,
            valinit=8,
        )

        # Create update callback for every slider's change:
        self.x_slider_current.on_changed(self.update)
        self.y_slider_current.on_changed(self.update)
        self.x_slider_next.on_changed(self.update)
        self.y_slider_next.on_changed(self.update)
        self.hip_slider.on_changed(self.update)
        self.knee_slider.on_changed(self.update)

        # Create a reset button for all sliders:
        ax_reset = plt.axes([0.7, 0.01, 0.25, 0.04])
        reset_button = Button(ax_reset, "Reset", hovercolor="0.975")
        reset_button.on_clicked(self.reset)

        # Create a toggle for rear ankle dorsi flexion reduction:
        ax_toggle = plt.axes([0.7, 0.07, 0.07, 0.04])
        self.toggle_df_rear = Button(ax_toggle, "df_r", color="green", hovercolor="red")
        self.toggle_df_rear.on_clicked(self.toggle_rear)

        # Create a toggle for front ankle dorsi flexion reduction:
        ax_toggle = plt.axes([0.79, 0.07, 0.07, 0.04])
        self.toggle_df_front = Button(
            ax_toggle, "df_f", color="green", hovercolor="red"
        )
        self.toggle_df_front.on_clicked(self.toggle_front)

        # Create a button to play animation:
        ax_play = plt.axes([0.88, 0.07, 0.07, 0.04])
        self.play = Button(ax_play, "play", color="green", hovercolor="green")
        self.play.on_clicked(self.interpolate_trajectory)

        # Show the widget:
        plt.show()

    # Interpolation:
    def interpolate_trajectory(self, event):
        if not self.playing:
            self.playing = True
            self.play.color = "red"
            self.play.hovercolor = "red"
            # Get joint angles and foot rotation of three poses:
            joint_angles = []
            foot_rotations = []
            for pose in self.poses:
                if pose["name"] == "current_pose":
                    joint_angles.append(pose["pose"].pose_left)
                    foot_rotations.append(
                        0
                    )  # zero for now, since there is nof rot_foot2
                else:
                    joint_angles.append(pose["pose"].pose_right)
                    foot_rotations.append(pose["pose"].rot_foot1)

            # Get interpolated points (linear for now):
            steps = np.linspace(0, 1, 21)
            interpolated_pose_list = []
            interpolated_foot_rotations = []

            for part in range(2):
                for step in steps:
                    interpolated_pose_list.append(
                        (1 - step) * np.array(joint_angles[part])
                        + step * np.array(joint_angles[part + 1])
                    )
                    interpolated_foot_rotations.append(
                        (1 - step) * foot_rotations[part]
                        + step * foot_rotations[part + 1]
                    )

            for i in range(len(interpolated_pose_list)):
                interpolated_pose = interpolated_pose_list[i]
                interpolated_foot_rotation = interpolated_foot_rotations[i]
                new_pose = Pose(interpolated_pose)
                new_pose.rot_foot1 = interpolated_foot_rotation

                positions = new_pose.calculate_joint_positions()
                positions = [pos - np.array([LENGTH_FOOT, 0]) for pos in positions]

                # Plot the pose:
                positions_x = [pos[0] for pos in positions]
                positions_y = [pos[1] for pos in positions]

                self.trajectory.set_xdata(positions_x)
                self.trajectory.set_ydata(positions_y)
                self.update(0)
                plt.pause(
                    0.001
                )  # Draw takes longer than pause, might consider to switch to pyqtgraph
            self.playing = False
            self.trajectory.set_xdata(0)
            self.trajectory.set_ydata(0)
            self.update(0)
            self.play.color = "green"
            self.play.hovercolor = "green"

    # The function to be called anytime a slider's value changes:
    def update(self, update_value):
        self.mid_pose["pose"] = deepcopy(self.current_pose["pose"])
        for pose in self.poses:

            # Get new pose:
            if pose["name"] == "current_pose":
                x = abs(self.x_slider_current.val)
                y = abs(self.y_slider_current.val)
            else:
                x = self.x_slider_next.val
                y = self.y_slider_next.val

            if pose["name"] == "mid_pose":
                pose["pose"].solve_mid_position(
                    x,
                    y,
                    0.5,
                    0.15,
                    "",
                )
            else:
                pose["pose"].reset_to_zero_pose()
                pose["pose"].solve_end_position(
                    x,
                    y,
                    "",
                    self.hip_slider.val,
                    np.deg2rad(self.knee_slider.val),
                    self.reduce_df_front,
                    self.reduce_df_rear,
                )

            # Update location data:
            positions = pose["pose"].calculate_joint_positions()

            # Translate current pose to have toes2 and in (0,0):
            if pose["name"] == "current_pose":
                positions = [pos - positions[-1] for pos in positions]
            else:
                positions = [pos - np.array([LENGTH_FOOT, 0]) for pos in positions]

            positions_x = [pos[0] for pos in positions]
            positions_y = [pos[1] for pos in positions]

            pose["plot"].set_xdata(positions_x)
            pose["plot"].set_ydata(positions_y)

            # Update goal locations:
            if pose["name"] == "next_pose":
                pose["goal_ankle"].set_xdata(x - LENGTH_FOOT)
                pose["goal_ankle"].set_ydata(y)
                pose["goal_toes"].set_xdata(x)
                pose["goal_toes"].set_ydata(y)

            # Update table with joint angles:
            pose_rad = pose["pose"].pose_right
            for i in np.arange(len(pose_rad)):
                self.table.get_celld()[(i + pose["start_cell"], 1)].get_text().set_text(
                    np.round(pose_rad[i], 2)
                )
                self.table.get_celld()[(i + pose["start_cell"], 2)].get_text().set_text(
                    np.round(np.rad2deg(pose_rad[i]), 2)
                )

        # Redraw plot:
        self.fig.canvas.draw_idle()

    # Reset function:
    def reset(self, event):
        self.x_slider_current.reset()
        self.y_slider_current.reset()
        self.x_slider_next.reset()
        self.y_slider_next.reset()
        self.hip_slider.reset()
        self.knee_slider.reset()

    # Toggle functions:
    def toggle_rear(self, event):
        if self.reduce_df_rear:
            self.reduce_df_rear = False
            self.toggle_df_rear.color = "red"
            self.toggle_df_rear.hovercolor = "green"
        else:
            self.reduce_df_rear = True
            self.toggle_df_rear.color = "green"
            self.toggle_df_rear.hovercolor = "red"
        self.update(0)

    def toggle_front(self, event):
        if self.reduce_df_front:
            self.reduce_df_front = False
            self.toggle_df_front.color = "red"
            self.toggle_df_front.hovercolor = "green"
        else:
            self.reduce_df_front = True
            self.toggle_df_front.color = "green"
            self.toggle_df_front.hovercolor = "red"
        self.update(0)


if __name__ == "__main__":
    widget = LiveWidget()
