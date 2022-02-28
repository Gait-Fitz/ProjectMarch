import PySimpleGUI as sg
from defusedxml import minidom
from importlib.machinery import SourceFileLoader
import subprocess

MAX = 100000000

# bash commands:
march_source = {
    "ros1": "source ~/march/ros1/install/local_setup.bash",
    "ros2": "source ~/march/ros2/install/local_setup.bash",
}
march_source["bridge"] = (
    march_source["ros1"]
    + " && "
    + march_source["ros2"]
    + " && "
    + "source ~/ros1_bridge/install/local_setup.bash"
)

ros_source = {
    "ros1": "source /opt/ros/noetic/local_setup.bash",
    "ros2": "source /opt/ros/foxy/local_setup.bash",
}
ros_source["bridge"] = ros_source["ros1"] + " && " + ros_source["ros2"]

default_launch_commands = {
    "ros1": "roslaunch march_launch march_simulation.launch",
    "ros2": "ros2 launch march_launch march_simulation.launch.py",
    "bridge": "export ROS_MASTER_URI=http://localhost:11311 && ros2 run ros1_bridge parameter_bridge",
}

# file locations:
path_ros1 = "ros1/src/march_launch/launch/march_simulation.launch"
path_ros2 = "ros2/src/march_launch/launch/march_simulation.launch.py"
name_ros2 = "march_simulation.launch"


class MarchLauncher:
    def __init__(self) -> None:
        self.ros_versions = ["ros1", "ros2"]
        self.default_arguments = {"ros1": [], "ros2": []}
        self.layouts = {"ros1": [], "ros2": []}
        self.launch_commands = {
            "ros1": "",
            "ros2": "",
            "bridge": default_launch_commands["bridge"],
        }

        self.load_ros1()
        self.load_ros2()

        self.read_ros1()
        self.read_ros2()

        self.fix_linked_values()
        self.fix_opposite_values()

        self.create_ros_layouts()
        self.design_window()
        self.show_window()

    def load_ros1(self):
        self.launch_file_ros1 = minidom.parse(path_ros1)

    def load_ros2(self):
        march_simulation_launch_package = SourceFileLoader(
            name_ros2, path_ros2
        ).load_module()

        self.launch_file_ros2 = (
            march_simulation_launch_package.generate_launch_description()
        )

    def read_ros1(self):
        args = self.launch_file_ros1.getElementsByTagName("arg")

        for arg in args:
            name = arg.attributes["name"].value

            if "doc" in arg.attributes:
                doc = arg.attributes["doc"].value
            else:
                doc = ""

            if "value" in arg.attributes:
                value = arg.attributes["value"].value
            else:
                value = arg.attributes["default"].value

            self.default_arguments["ros1"].append([name, value, doc])

    def read_ros2(self):
        for description in self.launch_file_ros2._LaunchDescription__entities[:-1]:
            name = description._DeclareLaunchArgument__name
            value_object = description._DeclareLaunchArgument__default_value[0]
            if hasattr(value_object, "_TextSubstitution__text"):
                value = value_object._TextSubstitution__text
            elif hasattr(value_object, "_LaunchConfiguration__variable_name"):
                value = (
                    "$(arg "
                    + value_object._LaunchConfiguration__variable_name[
                        0
                    ]._TextSubstitution__text
                    + ")"
                )
            else:
                print("Error")
            doc = description._DeclareLaunchArgument__description
            self.default_arguments["ros2"].append([name, value, doc])

    def fix_linked_values(self):

        for ros in self.ros_versions:
            updated_arguments = []

            for arg in self.default_arguments[ros]:
                name = arg[0]
                value = arg[1]
                doc = arg[2]
                while value.startswith("$(arg"):
                    for other_arg in self.default_arguments[ros]:
                        other_name = other_arg[0]
                        if other_name == value[6:-1]:
                            other_value = other_arg[1]
                            value = other_value
                updated_arguments.append([name, value, doc])
            self.default_arguments[ros] = updated_arguments

    def fix_opposite_values(self):

        for ros in self.ros_versions:
            updated_arguments = []

            for arg in self.default_arguments[ros]:
                name = arg[0]
                value = arg[1]
                doc = arg[2]
                if value.startswith("$(eval not"):
                    for other_arg in self.default_arguments[ros]:
                        other_name = other_arg[0]
                        other_value = other_arg[1]
                        if other_name == value[11:-1]:
                            if other_value == "false":
                                value = "true"
                            elif other_value == "true":
                                value = "false"
                updated_arguments.append([name, value, doc])
            self.default_arguments[ros] = updated_arguments

    def create_ros_layouts(self):
        for ros in self.ros_versions:

            arguments = self.default_arguments[ros]

            for arg in arguments:
                name, value, doc = arg

                if value in ["true", "True"]:
                    row = [
                        sg.Text(name, size=(20, 1)),
                        sg.Checkbox(
                            "",
                            default=True,
                            key=ros + "_" + name,
                            size=(17, 1),
                            enable_events=True,
                        ),
                        sg.Text(doc),
                    ]
                elif value in ["false", "False"]:
                    row = [
                        sg.Text(name, size=(20, 1)),
                        sg.Checkbox(
                            "",
                            default=False,
                            key=ros + "_" + name,
                            size=(17, 1),
                            enable_events=True,
                        ),
                        sg.Text(doc),
                    ]
                else:
                    row = [
                        sg.Text(name, size=(20, 1)),
                        sg.InputText(
                            value,
                            key=ros + "_" + name,
                            size=(20, 1),
                            enable_events=True,
                        ),
                        sg.Text(doc),
                    ]

                self.layouts[ros].append(row)

    def design_window(self):
        buttons_tools = [
            sg.Frame(
                "BRIDGE",
                [
                    [
                        sg.Button("RUN", key="run_bridge"),
                        sg.Button("BUILD", key="build_bridge"),
                        sg.Button("CLEAN", key="clean_bridge"),
                    ]
                ],
            ),
            sg.Frame(
                "TOOLS",
                [
                    [
                        sg.Button("RQT1", key="rqt1"),
                        sg.Button("RQT2", key="rqt2"),
                    ]
                ],
                size=(MAX, 50),
            ),
        ]
        buttons_ros1 = [
            sg.Frame(
                "ROS1",
                [
                    [
                        sg.Button("RUN", key="run_ros1"),
                        sg.Button("BUILD", key="build_ros1"),
                        sg.Button("CLEAN", key="clean_ros1"),
                    ]
                ],
            ),
            sg.Frame(
                "Launch command",
                [
                    [
                        sg.Button("RESET", key="reset_ros1"),
                        sg.Text(
                            default_launch_commands["ros1"], key="launch_argument_ros1"
                        ),
                    ]
                ],
                size=(MAX, 50),
            ),
        ]
        buttons_ros2 = [
            sg.Frame(
                "ROS2",
                [
                    [
                        sg.Button("RUN", key="run_ros2"),
                        sg.Button("BUILD", key="build_ros2"),
                        sg.Button("CLEAN", key="clean_ros2"),
                    ]
                ],
            ),
            sg.Frame(
                "Launch command",
                [
                    [
                        sg.Button("RESET", key="reset_ros2"),
                        sg.Text(
                            default_launch_commands["ros2"], key="launch_argument_ros2"
                        ),
                    ]
                ],
                size=(MAX, 50),
            ),
        ]
        frame_ros1 = sg.Frame(
            "ROS1",
            [[sg.Column(self.layouts["ros1"], size=(None, MAX), scrollable=True)]],
        )
        frame_ros2 = sg.Frame(
            "ROS2",
            [[sg.Column(self.layouts["ros2"], size=(None, MAX), scrollable=True)]],
        )

        self.window = sg.Window(
            "March Launcher",
            [
                [buttons_tools],
                [
                    buttons_ros1,
                ],
                [
                    buttons_ros2,
                ],
                [
                    frame_ros1,
                    frame_ros2,
                ],
            ],
            resizable=True,
        )

    def update_window(self, values):
        for ros in self.ros_versions:
            self.launch_commands[ros] = default_launch_commands[ros]
            for i in range(len(self.default_arguments[ros])):
                name = self.default_arguments[ros][i][0]
                default_value = self.default_arguments[ros][i][1]
                value = values[ros + "_" + name]

                if isinstance(value, bool):
                    value = str(value)
                    if ros == "ros1":
                        value = value.lower()
                if default_value != value:
                    self.launch_commands[ros] += " " + name + ":=" + value
            self.window["launch_argument_" + ros].update(self.launch_commands[ros])

    def run(self, ros: str):
        terminator_command = (
            'terminator -e "'
            + ros_source[ros]
            + " && "
            + march_source[ros]
            + " && "
            + self.launch_commands[ros]
            + '"'
        )
        print(terminator_command)
        subprocess.Popen([terminator_command], shell=True)

    def reset(self, ros: str):
        for arg in self.default_arguments[ros]:
            name, value, doc = arg
            if value in ["true", "True"]:
                value = True
            if value in ["false", "False"]:
                value = False
            self.window[ros + "_" + name].update(value)
            self.window["launch_argument_" + ros].update(default_launch_commands[ros])

    def clean(self, ros: str):
        sg.popup_yes_no(
            "Are you sure that you want to clean " + ros + "?",
            title="Clean " + ros,
            keep_on_top=True,
        )

    def show_window(self):
        while True:
            event, values = self.window.read()

            self.update_window(values)

            if event == "run_ros1":
                self.run("ros1")

            if event == "run_ros2":
                self.run("ros2")

            if event == "run_bridge":
                self.run("bridge")

            if event == "reset_ros1":
                self.reset("ros1")

            if event == "reset_ros2":
                self.reset("ros2")

            if event == "clean_ros1":
                self.clean("ros1")

            if event == "clean_ros2":
                self.clean("ros2")

            if event == sg.WIN_CLOSED:
                break

        self.window.close()


if __name__ == "__main__":
    march_launcher = MarchLauncher()
