"""Author: George Vegelien, MVII."""
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Loads and sets the default value for the parameters.

    The settable ros parameters are:
        robot (str): Which folder to use in loading the different PID values.
            The yaml files used can be found at 'config/{robot}/{configuration}.yaml'
        configuration (str): Which yaml file used in loading the different PID values.
            The yaml files used can be found at 'config/{robot}/{configuration}.yaml'
        slope (float): The value for how fast the old PID converges to the new PID value
            if they are different between gait types. The higher the value the faster it goes.
        linear (bool):  Boolean for if we want to slowly converge between PID values
            or set them instantly. `True` for converging, and `False` for instant.
    """
    # Used to get the correct subdirectory for the yaml config files.
    robot = LaunchConfiguration("robot")

    # Used to get the correct yaml config file, and is also logged in the RQT note taker
    configuration = LaunchConfiguration("configuration")

    yaml_param_config_file = [
        PathJoinSubstitution([get_package_share_directory("march_gain_scheduling"), "config", robot, configuration]),
        ".yaml",
    ]
    # e.g. of yaml_param_config_file: "ros2/src/control/march_gain_scheduling/config/{robot}/{configuration}.yaml"

    node_params = [
        yaml_param_config_file,
        {
            # Both used for the interpolation between two PID values.
            "linearize_gain_scheduling": LaunchConfiguration("linear"),
            "linear_slope": LaunchConfiguration("slope"),
            # Used to get the correct yaml config file, and is also logged in the RQT note taker
            "configuration": configuration,
        },
    ]

    actions = [
        DeclareLaunchArgument(
            name="linear", default_value="true", description="Whether to linearize the change in PID values"
        ),
        DeclareLaunchArgument(
            name="slope",
            default_value="100.0",
            description="The slope of the linear change in PID values. Only used when 'linear' is true",
        ),
        DeclareLaunchArgument(
            name="robot",
            default_value="march6",
            description="Robot to use. Options are the folders found in "
            "'ros2/src/control/march_gain_scheduling/config'",
        ),
        Node(
            name="march_gain_scheduling",
            package="march_gain_scheduling",
            executable="march_gain_scheduling_node",
            namespace="march",
            output="screen",
            parameters=node_params,
        ),
    ]

    return LaunchDescription(actions)
