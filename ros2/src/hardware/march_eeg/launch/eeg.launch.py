"""Author: George Vegelien, MVII."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument(name="headset", default_value="MOCK_Unicorn",
                              description="The headset used to gather the eeg data.",
                              choices=["NAUTILUS", "UNICORN", "MOCK_Unicorn"]),

        DeclareLaunchArgument(name="eeg_data_file_name", default_value="koen_data5.csv",
                              description="The file name used to gather the eeg data from,"
                                          "Should be located in '`package_share['march_eeg']`/resources/`file_name`'."
                                          "Note: if you use an actual headset it should be a live binary file,"
                                          "if you use a 'MOCK' then it should be a converted .csv file.",
                              choices=["koen_data5.csv", "[binary_file].bin"])
    ]

    node_params = [{
        "headset": LaunchConfiguration("headset"),
        "file_name": LaunchConfiguration("eeg_data_file_name"),
    }]

    eeg_nodes = [Node(
        name="march_eeg",
        package="march_eeg",
        executable="eeg_node",
        namespace="march",
        output="screen",
        parameters=node_params,
    )]

    return LaunchDescription(launch_arguments + eeg_nodes)
