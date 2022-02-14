import PySimpleGUI as sg
from defusedxml import minidom
from importlib.machinery import SourceFileLoader

MAX = 100000000


class MarchLauncher:
    def __init__(self) -> None:
        self.launch_file_ros1 = minidom.parse(
            "ros1/src/march_launch/launch/march_simulation.launch"
        )
        self.launch_file_ros2 = "march_simulation.launch"
        self.launch_path_ros2 = (
            "ros2/src/march_launch/launch/march_simulation.launch.py"
        )
        self.default_ros1_arguments = []
        self.default_ros2_arguments = []
        self.layout_ros1 = []
        self.layout_ros2 = []

        self.read_ros1()
        self.default_ros1_arguments = self.read_linked_values(
            self.default_ros1_arguments
        )
        self.default_ros1_arguments = self.read_opposite_values(
            self.default_ros1_arguments
        )

        self.read_ros2()
        self.default_ros2_arguments = self.read_linked_values(
            self.default_ros2_arguments
        )
        self.default_ros2_arguments = self.read_opposite_values(
            self.default_ros2_arguments
        )

        self.create_layout("ros1")
        self.create_layout("ros2")
        self.show_window()

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

            self.default_ros1_arguments.append([name, value, doc])

    def read_linked_values(self, arguments):
        updated_arguments = []

        for arg in arguments:
            name = arg[0]
            value = arg[1]
            doc = arg[2]
            while value.startswith("$(arg"):
                for other_arg in arguments:
                    other_name = other_arg[0]
                    if other_name == value[6:-1]:
                        other_value = other_arg[1]
                        value = other_value
            updated_arguments.append([name, value, doc])
        return updated_arguments

    def read_opposite_values(self, arguments):
        updated_arguments = []

        for arg in arguments:
            name = arg[0]
            value = arg[1]
            doc = arg[2]
            if value.startswith("$(eval not"):
                for other_arg in arguments:
                    other_name = other_arg[0]
                    other_value = other_arg[1]
                    if other_name == value[11:-1]:
                        if other_value == "false":
                            value = "true"
                        elif other_value == "true":
                            value = "false"
            updated_arguments.append([name, value, doc])

        return updated_arguments

    def create_layout(self, ros: str):
        if ros == "ros1":
            arguments = self.default_ros1_arguments
        elif ros == "ros2":
            arguments = self.default_ros2_arguments

        for arg in arguments:
            name, value, doc = arg

            if value in ["true", "True"]:
                row = [
                    sg.Text(name, size=(20, 1)),
                    sg.Checkbox(
                        "", default=True, key=name, size=(17, 1), enable_events=True
                    ),
                    sg.Text(doc),
                ]
            elif value in ["false", "False"]:
                row = [
                    sg.Text(name, size=(20, 1)),
                    sg.Checkbox(
                        "", default=False, key=name, size=(17, 1), enable_events=True
                    ),
                    sg.Text(doc),
                ]
            else:
                row = [
                    sg.Text(name, size=(20, 1)),
                    sg.InputText(value, key=name, size=(20, 1), enable_events=True),
                    sg.Text(doc),
                ]

            if ros == "ros1":
                self.layout_ros1.append(row)
            elif ros == "ros2":
                self.layout_ros2.append(row)

    def read_ros2(self):
        march_simulation_launch_package = SourceFileLoader(
            self.launch_file_ros2, self.launch_path_ros2
        ).load_module()

        launch_description = (
            march_simulation_launch_package.generate_launch_description()
        )

        for description in launch_description._LaunchDescription__entities[:-1]:
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
            self.default_ros2_arguments.append([name, value, doc])

    def show_window(self):
        default_launch_command = "roslaunch march_launch march_simulation.launch"
        frame_ros1 = sg.Frame(
            "ROS1", [[sg.Column(self.layout_ros1, size=(None, MAX), scrollable=True)]]
        )
        frame_ros2 = sg.Frame(
            "ROS2", [[sg.Column(self.layout_ros2, size=(None, MAX), scrollable=True)]]
        )
        buttons_ros1 = sg.Frame(
            "ROS1",
            [
                [
                    sg.Button("RUN", key="run_ros1"),
                    sg.Button("RESET", key="reset_ros1"),
                    sg.Text(default_launch_command, key="launch_argument_ros1"),
                ]
            ],
            size=(MAX, 50),
        )
        buttons_ros2 = sg.Frame(
            "ROS2",
            [
                [
                    sg.Button("RUN", key="run_ros2"),
                    sg.Button("RESET", key="reset_ros2"),
                    sg.Text(default_launch_command, key="launch_argument_ros1"),
                ]
            ],
            size=(MAX, 50),
        )

        window = sg.Window(
            "March Launcher",
            [
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

        # Create an event loop:
        while True:
            event, values = window.read()

            # Update launch argument:
            launch_command = default_launch_command
            for i in range(len(self.default_ros1_arguments)):
                name = self.default_ros1_arguments[i][0]
                default_value = self.default_ros1_arguments[i][1]
                value = values[name]

                if isinstance(value, bool):
                    value = str(value).lower()
                if default_value != value:
                    launch_command += " " + name + ":=" + value
            window["launch_argument_ros1"].update(launch_command)

            if event == "run_ros1":
                print("needs to be implementd!")

            if event == "reset_ros1":
                for arg in self.default_ros1_arguments:
                    name, value, doc = arg
                    if value in ["true", "True"]:
                        value = True
                    if value in ["false", "False"]:
                        value = False
                    window[name].update(value)
                    window["launch_argument_ros1"].update(default_launch_command)

            if event == sg.WIN_CLOSED:
                break

        window.close()


if __name__ == "__main__":
    march_launcher = MarchLauncher()
