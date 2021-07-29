import os
from typing import Dict
import traceback

import yaml
from ament_index_python.packages import get_package_share_directory
from march_gait_selection.gaits.balance_gait import BalanceGait
from march_gait_selection.gaits.dynamic_edge_setpoints_gait import (
    DynamicEdgeSetpointsGait,
)
from march_shared_msgs.srv import SetGaitVersion, ContainsGait, GetGaitParameters

from march_utility.exceptions.gait_exceptions import (
    GaitError,
    GaitNameNotFoundError,
    NonValidGaitContentError,
)
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from march_utility.utilities.node_utils import (
    get_robot_urdf_from_service,
    get_joint_names_from_robot,
)
from march_utility.utilities.utility_functions import (
    validate_and_get_joint_names_for_inverse_kinematics,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from urdf_parser_py import urdf

from march_gait_selection.gaits.realsense_gait import RealsenseGait
from march_gait_selection.gaits.setpoints_gait import SetpointsGait

NODE_NAME = "gait_selection"


class GaitSelection(Node):
    """Base class for the gait selection module."""

    def __init__(self, gait_package=None, directory=None, robot=None, balance=None):
        super().__init__(
            NODE_NAME, automatically_declare_parameters_from_overrides=True
        )
        self._balance_used = False
        try:
            # Initialize all parameters once, and set up a callback for dynamically
            # reconfiguring
            if gait_package is None:
                gait_package = (
                    self.get_parameter("gait_package")
                    .get_parameter_value()
                    .string_value
                )
            if directory is None:
                directory = (
                    self.get_parameter("gait_directory")
                    .get_parameter_value()
                    .string_value
                )
            if balance is None:
                self._balance_used = (
                    self.get_parameter("balance").get_parameter_value().bool_value
                )

            self._early_schedule_duration = self._parse_duration_parameter(
                "early_schedule_duration"
            )
            self._first_subgait_delay = self._parse_duration_parameter(
                "first_subgait_delay"
            )

        except ParameterNotDeclaredException:
            self.get_logger().error(
                "Gait selection node started without required parameters "
                "gait_package, gait_directory and balance"
            )

        self._directory_name = directory
        self._gait_package = gait_package
        self._gait_directory, self._default_yaml = self._initialize_gaits()
        if not os.path.isdir(self._gait_directory):
            self.get_logger().error(f"Gait directory does not exist: {directory}")
            raise FileNotFoundError(directory)
        if not os.path.isfile(self._default_yaml):
            self.get_logger().error(
                f"Gait default yaml file does not exist: {directory}/default.yaml"
            )

        self._robot = get_robot_urdf_from_service(self) if robot is None else robot
        self._joint_names = sorted(get_joint_names_from_robot(self._robot))

        self._realsense_yaml = os.path.join(
            self._gait_directory, "realsense_gaits.yaml"
        )

        self._realsense_gait_version_map = self._load_realsense_configuration()
        (
            self._gait_version_map,
            self._positions,
            self._dynamic_edge_version_map,
        ) = self._load_configuration()

        self._robot_description_sub = self.create_subscription(
            msg_type=String,
            topic="/march/robot_description",
            callback=self._update_robot_description_cb,
            qos_profile=10,
        )

        self._create_services()
        self._gaits = self._load_gaits()

        self._early_schedule_duration = self._parse_duration_parameter(
            "early_schedule_duration"
        )
        self._first_subgait_delay = self._parse_duration_parameter(
            "first_subgait_delay"
        )

        if not self._validate_inverse_kinematics_is_possible():
            self.get_logger().warn(
                "The currently available joints are unsuitable for "
                "using inverse kinematics.\n"
                "Any interpolation on foot_location will return "
                "the base subgait instead. Realsense gaits will "
                "not be loaded."
            )
        self.get_logger().info("Successfully initialized gait selection node.")

    @property
    def joint_names(self):
        return self._joint_names

    @property
    def gaits(self):
        return self._gaits

    def _validate_inverse_kinematics_is_possible(self):
        return (
            validate_and_get_joint_names_for_inverse_kinematics(self.get_logger())
            is not None
        )

    def _initialize_gaits(self):
        package_path = get_package_share_directory(self._gait_package)
        gait_directory = os.path.join(package_path, self._directory_name)
        default_yaml = os.path.join(gait_directory, "default.yaml")

        if not os.path.isdir(gait_directory):
            self.get_logger().error(
                f"Gait directory does not exist: " f"{gait_directory}"
            )
        if not os.path.isfile(default_yaml):
            self.get_logger().error(
                f"Gait default yaml file does not exist: "
                f"{gait_directory}/default.yaml"
            )
        return gait_directory, default_yaml

    def update_gaits(self):
        """
        Update the gaits after one of the gait attributes has been changed.
        """
        self._gait_directory, self._default_yaml = self._initialize_gaits()
        self._realsense_yaml = os.path.join(
            self._gait_directory, "realsense_gaits.yaml"
        )

        self._realsense_gait_version_map = self._load_realsense_configuration()
        (
            self._gait_version_map,
            self._positions,
            self._semi_dynamic_gait_version_map,
        ) = self._load_configuration()

        self._loaded_gaits = self._load_gaits()

    def _create_services(self) -> None:
        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_version_map",
            callback=lambda req, res: Trigger.Response(
                success=True, message=str(self.gait_version_map)
            ),
        )

        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_gait_directory",
            callback=lambda req, res: Trigger.Response(
                success=True, message=self._directory_name
            ),
        )

        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_default_dict",
            callback=self.get_default_dict_cb,
        )

        self.create_service(
            srv_type=SetGaitVersion,
            srv_name="/march/gait_selection/set_gait_version",
            callback=self.set_gait_versions_cb,
        )

        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_directory_structure",
            callback=lambda req, res: Trigger.Response(
                success=True, message=str(self.scan_directory())
            ),
        )

        self.create_service(
            srv_type=ContainsGait,
            srv_name="/march/gait_selection/contains_gait",
            callback=self.contains_gait_cb,
        )

    def _parse_duration_parameter(self, name: str) -> Duration:
        """Get a duration parameter from the parameter server.

        Returns 0 if the parameter does not exist.
        Clamps the duration to 0 if it is negative.
        """
        if self.has_parameter(name):
            value = self.get_parameter(name).value
            if value < 0:
                value = 0
            duration = Duration(seconds=value)
        else:
            duration = Duration(0)
        return duration

    def shortest_subgait(self) -> Subgait:
        """Get the subgait with the smallest duration of all subgaits in the loaded gaits."""
        shortest_subgait = None
        for gait in self._gaits.values():
            for subgait in gait.subgaits.values():
                if (
                    shortest_subgait is None
                    or subgait.duration < shortest_subgait.duration
                ):
                    shortest_subgait = subgait
        return shortest_subgait

    @property
    def robot(self):
        """Return the robot obtained from the robot state publisher."""
        return self._robot

    @property
    def gait_version_map(self):
        """Returns the mapping from gaits and subgaits to versions."""
        return self._gait_version_map

    @property
    def positions(self):
        """Returns the named idle positions."""
        return self._positions

    def _update_robot_description_cb(self, msg):
        """
        Callback that is used to update the robot description when
        robot_state_publisher sends out an update.
        """
        self._robot = urdf.Robot.from_xml_string(msg.data)

    def set_gait_versions(self, gait_name, version_map):
        """Sets the subgait versions of given gait.

        :param str gait_name: Name of the gait to change versions
        :param dict version_map: Mapping subgait names to versions
        """
        if gait_name not in self._gaits:
            raise GaitNameNotFoundError(gait_name)

        # Only update versions that are different
        version_map = {
            name: version
            for name, version in version_map.items()
            if version != self._gait_version_map[gait_name]["subgaits"][name]
        }
        self._gaits[gait_name].set_subgait_versions(
            self._robot, self._gait_directory, version_map
        )
        self._gait_version_map[gait_name].update(version_map)
        self.get_logger().info(
            f"Setting gait versions successful: {self._gaits[gait_name]}"
        )

    def set_gait_versions_cb(self, request, response):
        """Sets a new gait version to the gait selection instance.

        :type msg: march_shared_resources.srv.SetGaitVersionRequest

        :rtype march_shared_resources.srv.SetGaitVersionResponse
        """

        if len(request.subgaits) != len(request.versions):
            return [False, "`subgaits` and `versions` array are not of equal length"]

        version_map = dict(zip(request.subgaits, request.versions))
        try:
            self.get_logger().info(
                f"Setting gait versions for gait {request.gait} with "
                f"version map {version_map}"
            )
            self.set_gait_versions(request.gait, version_map)
            response.success = True
            response.message = ""
            return response
        except Exception:  # noqa: PIE786
            response.success = False
            response.message = (
                "Something went wrong when setting the gait version"
                + str(traceback.format_exc())  # noqa: W503
            )
            return response

    def contains_gait_cb(self, request, response):
        """
        Checks whether a gait and subgait are loaded.

        :type request: ContainsGaitRequest
        :param request: service request
        :return: True when the gait and subgait are loaded
        """
        gait = self._gaits.get(request.gait)
        if gait is None:
            response.contains = False
            return response

        response.contains = True
        for subgait in request.subgaits:
            if gait[subgait] is None:
                response.contains = False
        return response

    def scan_directory(self):
        """Scans the gait_directory recursively and create a dictionary of all
        subgait files.

        :returns:
            dictionary of the maps and files within the directory
        """
        gaits = {}
        for gait in os.listdir(self._gait_directory):
            gait_path = os.path.join(self._gait_directory, gait)

            if os.path.isdir(gait_path):
                subgaits = {}

                for subgait in os.listdir(gait_path):
                    subgait_path = os.path.join(gait_path, subgait)

                    if os.path.isdir(subgait_path):
                        versions = sorted(
                            [
                                v.replace(".subgait", "")
                                for v in os.listdir(os.path.join(subgait_path))
                                if v.endswith(".subgait")
                            ]
                        )
                        subgaits[subgait] = versions

                gaits[gait] = subgaits
        return gaits

    def get_default_dict_cb(self, req, res):
        defaults = {"gaits": self._gait_version_map, "positions": self._positions}
        return Trigger.Response(success=True, message=str(defaults))

    def add_gait(self, gait):
        """Adds a gait to the loaded gaits if it does not already exist.

        The to be added gait should implement `GaitInterface`.
        """
        if gait.name in self._gaits:
            self.get_logger().warn(
                "Gait `{gait}` already exists in gait selection".format(gait=gait.name)
            )
        else:
            self._gaits[gait.name] = gait

    def _load_gaits(self):
        """Loads the gaits in the specified gait directory.

        :returns dict: A dictionary mapping gait name to gait instance
        """
        gaits = {}

        for gait_name in self._gait_version_map:
            gaits[gait_name] = SetpointsGait.from_file(
                gait_name,
                self._gait_directory,
                self._robot,
                self._gait_version_map,
                self._gait_path_to_read_map[gait_name],
            )

        for gait_name in self._dynamic_edge_version_map:
            self.get_logger().debug(f"Adding dynamic gait {gait_name}")
            gaits[gait_name] = DynamicEdgeSetpointsGait.from_file(
                gait_name,
                self._gait_directory,
                self._robot,
                self._dynamic_edge_version_map,
                self._gait_path_to_read_map[gait_name],
            )
            self._gait_version_map[gait_name] = self._dynamic_edge_version_map[
                gait_name
            ]
        self._load_realsense_gaits(gaits)
        if self._balance_used and "balance_walk" in gaits.keys():
            balance_gait = BalanceGait(node=self, default_walk=gaits["balance_walk"])
            if balance_gait is not None:
                self.get_logger().info("Successfully created a balance gait")
                gaits["balanced_walk"] = balance_gait

        return gaits

    def _load_realsense_gaits(self, gaits):
        """
        Load all gaits from the realsense gait version map.
        Also create a service with a separate callback group that can be used by the
        realsense gaits to get parameters from the realsense_reader. A new callback
        group is necessary to prevent a deadlock.

        :param gaits: The dictionary where the loaded gaits will be added to.
        """
        if not self._validate_inverse_kinematics_is_possible():
            return
        get_gait_parameters_service = self.create_client(
            srv_type=GetGaitParameters,
            srv_name="/camera/process_pointcloud",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        for gait_name in self._realsense_gait_version_map:
            gait_folder = gait_name
            gait_path = os.path.join(
                self._gait_directory, gait_folder, gait_name + ".gait"
            )
            with open(gait_path, "r") as gait_file:
                gait_graph = yaml.load(gait_file, Loader=yaml.SafeLoader)["subgaits"]
            gait = RealsenseGait.from_yaml(
                gait_selection=self,
                robot=self._robot,
                gait_name=gait_name,
                gait_config=self._realsense_gait_version_map[gait_name],
                gait_graph=gait_graph,
                gait_directory=self._gait_directory,
                process_service=get_gait_parameters_service,
                gait_path_to_read=self._gait_path_to_read_map[gait_name],
            )
            gaits[gait_name] = gait

    def _load_realsense_configuration(self):
        if not os.path.isfile(self._realsense_yaml):
            self.get_logger().info(
                "No realsense_yaml present, no realsense gaits will be created."
            )
            return {}
        with open(self._realsense_yaml, "r") as realsense_config_file:
            realsense_config = yaml.load(realsense_config_file, Loader=yaml.SafeLoader)
        return realsense_config

    def _load_configuration(self):
        """Loads and verifies the gaits configuration."""
        with open(self._default_yaml, "r") as default_yaml_file:
            default_config = yaml.load(default_yaml_file, Loader=yaml.SafeLoader)

        version_map = default_config["gaits"]
        dynamic_edge_version_map = {}
        if "dynamic_edge_gaits" in default_config.keys():
            dynamic_edge_version_map = default_config["dynamic_edge_gaits"]

        if not isinstance(version_map, dict):
            raise TypeError("Gait version map should be of type; dictionary")

        self._gait_path_to_read_map = self._get_gait_path_to_read_map(
            {**version_map, **self._realsense_gait_version_map}
        )

        if not self._validate_version_map(version_map):
            raise GaitError(
                msg="Gait version map: {gm}, is not valid".format(gm=version_map)
            )

        positions = {}

        for position_name, position_values in default_config["positions"].items():
            positions[position_name] = {
                "gait_type": position_values["gait_type"],
                "joints": {},
            }
            for joint, joint_value in position_values["joints"].items():
                if joint in self._joint_names:
                    positions[position_name]["joints"][joint] = joint_value

            if set(positions[position_name]["joints"].keys()) != set(self._joint_names):
                raise NonValidGaitContentError(
                    f"The position {position_name} does not "
                    f"have a position for all required joints: it "
                    f"has {positions[position_name]['joints'].keys()}, "
                    f"required: {self._joint_names}"
                )
        return version_map, positions, dynamic_edge_version_map

    def _get_gait_path_to_read_map(self, version_map: Dict):
        gait_to_read_paths_map = {}
        for gait_name in version_map:
            gait_to_read_paths_map[
                gait_name
            ] = self._get_gait_directory_and_name_to_read_from(version_map, gait_name)
        return gait_to_read_paths_map

    def _validate_version_map(self, version_map):
        """Validates if the current versions exist.

        :param dict version_map: Version map to verify
        :returns bool: True when all versions exist, False otherwise
        """
        for gait_name in version_map:
            gait_path = os.path.join(self._gait_directory, gait_name)
            if not os.path.isfile(os.path.join(gait_path, gait_name + ".gait")):
                self.get_logger().warn("gait {gn} does not exist".format(gn=gait_name))
                return False

            for subgait_name in version_map[gait_name]["subgaits"]:
                version = version_map[gait_name]["subgaits"][subgait_name]
                if not Subgait.validate_version(
                    self._gait_path_to_read_map[gait_name], subgait_name, version
                ):
                    self.get_logger().warn(
                        f"{subgait_name}, {version} does not exist for gait {gait_name}"
                        f" which should be read from "
                        f"{self._gait_path_to_read_map[gait_name]}"
                    )
                    return False
        return True

    def _get_gait_directory_and_name_to_read_from(self, version_map, gait_name):
        gait_directory_to_read_from = version_map[gait_name].get(
            "reads_from_directory", None
        )
        if gait_directory_to_read_from is None:
            gait_directory_to_read_from = self._gait_directory
        else:
            gait_directory_to_read_from = os.path.join(
                os.path.dirname(self._gait_directory), gait_directory_to_read_from
            )

        gait_name_to_read_from = version_map[gait_name].get(
            "reads_from_gait", gait_name
        )
        return os.path.join(gait_directory_to_read_from, gait_name_to_read_from)

    def __getitem__(self, name):
        """Returns a gait from the loaded gaits."""
        return self._gaits.get(name)

    def __iter__(self):
        """Returns an iterator over all loaded gaits."""
        return iter(self._gaits.values())
