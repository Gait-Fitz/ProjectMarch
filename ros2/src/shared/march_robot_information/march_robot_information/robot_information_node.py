from typing import List, Optional

import rclpy
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from march_shared_msgs.srv import GetJointNames
from rclpy.node import Node
from urdf_parser_py import urdf

NODE_NAME = 'robot_information_node'


def main():
    """Starts the robot information node."""
    rclpy.init()

    node = RobotInformation()

    rclpy.spin(node)


class RobotInformation(Node):
    """The RobotInformation is a simple node that holds additional information
    about the march robot in its parameters."""
    def __init__(self, joint_names: Optional[List[str]] = None):
        super().__init__(NODE_NAME,
                         automatically_declare_parameters_from_overrides=True)
        self.get_parameter_clients = {}

        if joint_names is not None:
            self.joint_names = joint_names
        else:
            self.joint_names = self.get_joint_names()

        self.create_service(GetJointNames, 'robot_information/get_joint_names',
                            self.get_joint_names_cb)

    def get_joint_names(self) -> List[str]:
        """
        Add the list of joint names to the parameters of this node.

        First queries the robot_description from the robot_state_publisher,
        then finds the non-fixed joints in the urdf and stores them in a list.
        Finally this list is set as the parameter value for 'joint_names'
        """
        robot_description = self.make_get_parameters_request(
            node='/march/robot_state_publisher',
            names=['robot_description', 'abcd'])[0].string_value
        robot = urdf.Robot.from_xml_string(robot_description)

        joint_names = []
        for joint in robot.joints:
            if joint.type != "fixed":
                joint_names.append(joint.name)

        self.declare_parameter(name='joint_names', value=joint_names)
        return joint_names

    def make_get_parameters_request(self, node: str, names: List[str]) \
            -> List[ParameterValue]:
        """
        Make a request to a GetParameters service of a node.
        :param node: Node to make the request to.
        :param names: Parameter names to request from the node.
        :return: Returns the values that are retrieved from the service call.
        """
        srv_name = f'{node}/get_parameters'
        if srv_name not in self.get_parameter_clients:
            client = self.create_client(GetParameters, srv_name)
            self.get_parameter_clients[srv_name] = client
        else:
            client = self.get_parameter_clients[srv_name]

        while not client.wait_for_service(timeout_sec=1):
            self.get_logger().info(
                f"Waiting for {srv_name} to become available.")

        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, future)

        return future.result().values

    def get_joint_names_cb(self, _, response):
        """Return the joint names."""
        response.joint_names = self.joint_names
        return response
