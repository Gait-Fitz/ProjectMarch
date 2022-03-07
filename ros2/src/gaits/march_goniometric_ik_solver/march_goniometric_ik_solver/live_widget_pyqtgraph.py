import pyqtgraph as pg
import numpy as np
import sys
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QSlider, QWidget, QGridLayout, QPushButton
from march_goniometric_ik_solver.ik_solver import Pose, LENGTH_HIP
import live_widget_node
import rclpy

DEFAULT_HIP_FRACTION = 0.5
DEFAULT_KNEE_BEND = np.deg2rad(8)
REDUCE_DF_REAR = False
REDUCE_DF_FRONT = False

X_MIN = 0.0
X_MAX = 0.6
Y_MIN = -0.3
Y_MAX = 0.3

JOINT_NAMES = [
    "ankle1",
    "hip1_aa",
    "hip1_fe",
    "knee1",
    "ankle2",
    "hip2_aa",
    "hip2_fe",
    "knee2",
]


class LiveWidget:
    def __init__(self) -> None:
        self.sliders = {"last": {"x": 0, "y": 0}, "next": {"x": 0, "y": 0}}

        self.create_window()
        self.create_plot()
        self.create_sliders()
        self.create_buttons()
        self.create_table()
        self.fill_layout()

        # Create a timer object that is used to update the pose in live modus:
        self.live = False
        self.live_updater = QTimer()
        self.live_updater.setInterval(100)
        self.live_updater.timeout.connect(self.update_live)

    def create_window(self):
        self.window = QWidget()
        self.window.setWindowTitle("IK Solver - Widget")
        self.layout = QGridLayout(self.window)
        pg.setConfigOptions(antialias=True)

    def create_plot(self):
        self.plot_window = pg.GraphicsWindow()
        self.plot_window.setBackground("w")
        plot = self.plot_window.addPlot()
        plot.setAspectLocked()
        plot.showGrid(x=True, y=True)

        self.poses = {"last": Pose(), "next": Pose()}
        self.plots = {
            "last": plot.plot(pen="b", symbol="o", symbolSize=6),
            "next": plot.plot(pen="k", symbol="o", symbolSize=6),
        }
        self.update_poses()

    def create_sliders(self):
        self.slider_last_x = QSlider()
        self.slider_last_x.setOrientation(Qt.Horizontal)
        self.slider_last_x.setValue(99)
        self.slider_last_x.valueChanged.connect(self.update_last_x)

        self.slider_next_x = QSlider()
        self.slider_next_x.setOrientation(Qt.Horizontal)
        self.slider_next_x.valueChanged.connect(self.update_next_x)

        self.horizontal_sliders = QGridLayout()
        self.horizontal_sliders.addWidget(self.slider_last_x, 0, 0)
        self.horizontal_sliders.addWidget(self.slider_next_x, 0, 1)

        self.slider_last_y = QSlider()
        self.slider_last_y.setValue(50)
        self.slider_last_y.setOrientation(Qt.Vertical)
        self.slider_last_y.valueChanged.connect(self.update_last_y)

        self.slider_next_y = QSlider()
        self.slider_next_y.setValue(50)
        self.slider_next_y.setOrientation(Qt.Vertical)
        self.slider_next_y.valueChanged.connect(self.update_next_y)

        self.vertical_sliders = QGridLayout()
        self.vertical_sliders.addWidget(self.slider_last_y, 0, 0)
        self.vertical_sliders.addWidget(self.slider_next_y, 0, 1)

    def create_buttons(self):
        self.buttons = QGridLayout()
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset)
        self.toggle_button = QPushButton("Start Live")
        self.toggle_button.clicked.connect(self.toggle_live)
        self.buttons.addWidget(self.reset_button, 0, 0)
        self.buttons.addWidget(self.toggle_button, 0, 1)

    def toggle_live(self):
        self.slider_last_x.setHidden(not self.slider_last_x.isHidden())
        self.slider_last_y.setHidden(not self.slider_last_y.isHidden())
        self.slider_next_x.setHidden(not self.slider_next_x.isHidden())
        self.slider_next_y.setHidden(not self.slider_next_y.isHidden())
        self.reset()

        if self.live:
            self.toggle_button.setText("Start Live")
            self.live_updater.stop()
            rclpy.shutdown()
        else:
            self.toggle_button.setText("Stop Live")
            rclpy.init(args=None)
            self.subscriber = live_widget_node.LiveWidgetSubscriber(self.callback_live)
            self.live_updater.start()

        self.live = not self.live

    def update_live(self):
        self.message_recieved = False
        rclpy.spin_once(self.subscriber, timeout_sec=1)
        if not self.message_recieved:
            print("no message recieved...")

    def callback_live(self, msg):
        self.message_recieved = True
        self.joint_positions = msg.actual.positions
        pose = Pose(self.joint_positions)
        positions = pose.calculate_joint_positions()

        # shift positions to have toes of stand legs at (0,0):
        positions = [pos - positions[0] for pos in positions]

        positions_x = [pos[0] for pos in positions]
        positions_y = [pos[1] for pos in positions]
        self.plots["next"].setData(x=positions_x, y=positions_y)

    def create_table(self):
        self.table = QGridLayout()
        self.tables = {"last": pg.TableWidget(), "next": pg.TableWidget()}
        self.update_tables()
        for pose in ["last", "next"]:
            self.table.addWidget(
                self.tables[pose], list(self.tables.keys()).index(pose), 0
            )

    def fill_layout(self):
        self.layout.addLayout(self.vertical_sliders, 0, 0)
        self.layout.addWidget(self.plot_window, 0, 1)
        self.layout.addLayout(self.horizontal_sliders, 1, 1)
        self.layout.addLayout(self.table, 0, 2)
        self.layout.addLayout(self.buttons, 1, 2)

    def update_last_x(self, value):
        self.sliders["last"]["x"] = (1 - (value / 99)) * (X_MAX - X_MIN) + X_MIN
        self.update_pose("last")
        self.update_tables()

    def update_next_x(self, value):
        self.sliders["next"]["x"] = (value / 99) * (X_MAX - X_MIN) + X_MIN
        self.update_pose("next")
        self.update_tables()

    def update_last_y(self, value):
        self.sliders["last"]["y"] = (1 - (value / 99)) * (Y_MAX - Y_MIN) + Y_MIN
        self.update_pose("last")
        self.update_tables()

    def update_next_y(self, value):
        self.sliders["next"]["y"] = (value / 99) * (Y_MAX - Y_MIN) + Y_MIN
        self.update_pose("next")
        self.update_tables()

    def reset(self):
        self.slider_last_x.setValue(99)
        self.slider_next_x.setValue(0)
        self.slider_last_y.setValue(50)
        self.slider_next_y.setValue(50)
        for pose in ["last", "next"]:
            for axis in ["x", "y"]:
                self.sliders[pose][axis] = 0
        self.update_poses()
        self.update_tables()

    def update_pose(self, pose):
        self.poses[pose].solve_end_position(
            self.sliders[pose]["x"],
            self.sliders[pose]["y"],
            LENGTH_HIP,
            "",
            DEFAULT_HIP_FRACTION,
            DEFAULT_KNEE_BEND,
            REDUCE_DF_FRONT,
            REDUCE_DF_REAR,
        )
        positions = self.poses[pose].calculate_joint_positions()

        # shift positions to have toes of stand legs at (0,0):
        if pose == "last":
            positions = [pos - positions[-1] for pos in positions]
        elif pose == "next":
            positions = [pos - positions[0] for pos in positions]

        positions_x = [pos[0] for pos in positions]
        positions_y = [pos[1] for pos in positions]
        self.plots[pose].setData(x=positions_x, y=positions_y)

    def update_poses(self):
        for pose in ["last", "next"]:
            self.update_pose(pose)

    def update_tables(self):
        for pose in ["last", "next"]:
            joint_angles = self.poses[pose].pose_left
            joint_angles_degrees = [np.rad2deg(angle) for angle in joint_angles]

            data = []
            for joint, angle_rad, angle_deg in zip(
                JOINT_NAMES,
                np.round(joint_angles, 3),
                np.round(joint_angles_degrees, 3),
            ):
                data.append([joint, angle_rad, angle_deg])

            self.tables[pose].setData(data)
            self.tables[pose].setHorizontalHeaderLabels(["", "rad", "deg"])
            self.tables[pose].verticalHeader().hide()

    def show(self):
        self.window.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    live_widget = LiveWidget()
    live_widget.show()
    sys.exit(app.exec_())
