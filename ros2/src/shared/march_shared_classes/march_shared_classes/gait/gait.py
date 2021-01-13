import os

import yaml
from urdf_parser_py import urdf

from march_shared_classes.exceptions.gait_exceptions import (
    GaitNameNotFound,
    NonValidGaitContent,
    SubgaitNameNotFound,
)

from .subgait import Subgait
from .subgait_graph import SubgaitGraph

ALLOWED_ERROR_ENDPOINTS = 0.0001


class Gait(object):
    """base class for a generated gait."""

    def __init__(self, gait_name: str, subgaits: dict, graph: SubgaitGraph):
        """Initializes and verifies the gait.

        :param str gait_name: Name of the gait
        :param dict subgaits: Mapping of names to subgait instances
        :param SubgaitGraph graph: Mapping of subgait names transitions
        """
        self.gait_name = gait_name
        self.subgaits = subgaits
        self.graph = graph

        self._validate_trajectory_transition()

    @classmethod
    def from_file(
        cls,
        gait_name: str,
        gait_directory: str,
        robot: urdf.Robot,
        gait_version_map: dict,
    ):
        """Extract the data from the .gait file.

        :param gait_name:
            name of the gait to unpack
        :param gait_directory:
            path of the directory where the .gait file is located
        :param robot:
            the robot corresponding to the given .gait file
        :param gait_version_map:
            The parsed yaml file which states the version of the subgaits
        """
        gait_folder = gait_name
        gait_path = os.path.join(gait_directory, gait_folder, gait_name + ".gait")
        with open(gait_path, "r") as gait_file:
            gait_dictionary = yaml.load(gait_file, Loader=yaml.SafeLoader)

        return cls.from_dict(robot, gait_dictionary, gait_directory, gait_version_map)

    @classmethod
    def from_dict(
        cls,
        robot: urdf.Robot,
        gait_dictionary: dict,
        gait_directory: str,
        gait_version_map: dict,
    ):
        """Create a new gait object using the .gait and .subgait files.

        :param robot:
            the robot corresponding to the given .gait file
        :param gait_dictionary:
            the information of the .gait file as a dictionary
        :param gait_directory:
            path of the directory where the .gait file is located
        :param gait_version_map:
            The parsed yaml file which states the version of the subgaits

        :return:
            If the data in the files is validated a gait object is returned
        """
        gait_name = gait_dictionary["name"]
        subgaits = gait_dictionary["subgaits"]

        graph = SubgaitGraph(subgaits)
        subgaits = dict(
            [
                (
                    name,
                    cls.load_subgait(
                        robot, gait_directory, gait_name, name, gait_version_map
                    ),
                )
                for name in subgaits
                if name not in ("start", "end")
            ]
        )
        return cls(gait_name, subgaits, graph)

    @staticmethod
    def load_subgait(
        robot: urdf.Robot,
        gait_directory: str,
        gait_name: str,
        subgait_name: str,
        gait_version_map: dict,
    ) -> Subgait:
        """Read the .subgait file and extract the data.
        :param robot: the robot corresponding to the given .gait file
        :param gait_directory: path of the directory where the .gait file is located
        :param gait_name: the name of the gait where the subgait belongs to
        :param subgait_name: the name of the subgait to load
        :param gait_version_map: the parsed yaml file which states the version of
        the subgaits
        :return: Gait if gait and subgait names are valid return populated Gait object
        """
        if gait_name not in gait_version_map:
            raise GaitNameNotFound(gait_name)
        if subgait_name not in gait_version_map[gait_name]:
            raise SubgaitNameNotFound(subgait_name, gait_name)

        version = gait_version_map[gait_name][subgait_name]
        return Subgait.from_name_and_version(
            robot, gait_directory, gait_name, subgait_name, version
        )

    def _validate_trajectory_transition(self):
        """Compares and validates the trajectory end and start points."""
        for from_subgait_name, to_subgait_name in self.graph:
            if (
                len(
                    {from_subgait_name, to_subgait_name}
                    & {self.graph.START, self.graph.END}
                )
                > 0
            ):
                continue

            from_subgait = self.subgaits[from_subgait_name]
            to_subgait = self.subgaits[to_subgait_name]

            if not from_subgait.validate_subgait_transition(to_subgait):
                raise NonValidGaitContent(
                    msg="Gait {gait} with end setpoint of subgait {sn} to subgait {ns} "
                    "does not match".format(
                        gait=self.gait_name,
                        sn=from_subgait.subgait_name,
                        ns=to_subgait.subgait_name,
                    )
                )

    def set_subgait_versions(
        self, robot: urdf.Robot, gait_directory: str, version_map: dict
    ):
        """Updates the given subgait versions and verifies transitions.

        :param robot: URDF matching subgaits
        :param str gait_directory: path to the gait directory
        :param dict version_map: Mapping subgait names to versions
        """
        new_subgaits = {}
        for subgait_name, version in version_map.items():
            if subgait_name not in self.subgaits:
                raise SubgaitNameNotFound(subgait_name, self.gait_name)
            new_subgaits[subgait_name] = Subgait.from_name_and_version(
                robot, gait_directory, self.gait_name, subgait_name, version
            )

        for from_subgait_name, to_subgait_name in self.graph:
            if from_subgait_name in new_subgaits or to_subgait_name in new_subgaits:
                if from_subgait_name == self.graph.START:
                    old_subgait = self.subgaits[to_subgait_name]
                    new_subgait = new_subgaits[to_subgait_name]

                    old_starting_positions = old_subgait.starting_position
                    new_starting_positions = new_subgait.starting_position
                    for joint in old_subgait.joints:
                        if (
                            abs(
                                old_starting_positions[joint.name]
                                - new_starting_positions[joint.name]
                            )
                            >= ALLOWED_ERROR_ENDPOINTS
                        ):
                            raise NonValidGaitContent(
                                msg=f"The starting position of new version "
                                f"{self.gait_name} {to_subgait_name} does not match"
                            )
                elif to_subgait_name == self.graph.END:
                    old_subgait = self.subgaits[from_subgait_name]
                    new_subgait = new_subgaits[from_subgait_name]

                    old_final_positions = old_subgait.final_position
                    new_final_positions = new_subgait.final_position
                    for joint in old_subgait.joints:
                        if (
                            abs(
                                old_final_positions[joint.name]
                                - new_final_positions[joint.name]
                            )
                            >= ALLOWED_ERROR_ENDPOINTS
                        ):
                            raise NonValidGaitContent(
                                msg=f"The final position of new version "
                                f"{self.gait_name} {from_subgait_name} does not "
                                f"match"
                            )

                else:
                    from_subgait = new_subgaits.get(
                        from_subgait_name, self.subgaits[from_subgait_name]
                    )
                    to_subgait = new_subgaits.get(
                        to_subgait_name, self.subgaits[to_subgait_name]
                    )

                    if not from_subgait.validate_subgait_transition(to_subgait):
                        raise NonValidGaitContent(
                            msg=f"Gait {self.gait_name} with end setpoint of subgait "
                            f"{from_subgait_name.subgait_name} to "
                            f"subgait {to_subgait.subgait_name} does not match"
                        )

        self.subgaits.update(new_subgaits)

    def __getitem__(self, name: str):
        """Returns a subgait from the loaded subgaits."""
        return self.subgaits.get(name)
