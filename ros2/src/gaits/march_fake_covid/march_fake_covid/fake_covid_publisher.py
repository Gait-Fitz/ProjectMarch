# Ignore for flake8 because it errors on the RNG
# flake8: noqa
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Point
import random

NODE_NAME = "fake_covid_publisher"


class FakeCovidPublisher(Node):
    def __init__(self):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)
        self.random_x = False
        self.random_y = False

        self.location_x = (
            self.get_parameter("location_x").get_parameter_value().double_value
        )
        self.location_y = (
            self.get_parameter("location_y").get_parameter_value().double_value
        )
        self.location_z = (
            self.get_parameter("location_z").get_parameter_value().double_value
        )

        self.left_foot_publisher = self.create_publisher(
            Point,
            "/foot_position/left",
            10,
        )

        self.right_foot_publisher = self.create_publisher(
            Point,
            "/foot_position/right",
            10,
        )

        self.create_timer(0.1, self.publish_locations)

    def publish_locations(self):
        point = Point()

        if self.random_x:
            point.x = random.uniform(0.2, 0.5)
        else:
            point.x = self.location_x

        if self.random_y:
            point.y = random.uniform(0.0, 0.2)
        else:
            point.y = self.location_y

        point.z = self.location_z

        self.left_foot_publisher.publish(point)
        self.right_foot_publisher.publish(point)


def main():
    rclpy.init()
    fake_covid_publisher = FakeCovidPublisher()

    fake_covid_publisher.add_on_set_parameters_callback(
        lambda params: parameter_callback(fake_covid_publisher, params)
    )

    rclpy.spin(fake_covid_publisher)

    rclpy.shutdown()

def parameter_callback(fake_covid_publisher, parameters):
    for param in parameters:
        if param.name == "location_x":
            if param.value == "random":
                fake_covid_publisher.random_x = True
                fake_covid_publisher.get_logger().info("x set to random.")
            else:
                fake_covid_publisher.random_x = False
                fake_covid_publisher.location_x = param.value
                fake_covid_publisher.get_logger().info(f"x set to {param.value}")
        if param.name == "location_y":
            if param.value == "random":
                fake_covid_publisher.random_y = True
                fake_covid_publisher.get_logger().info("y set to random.")
            else:
                fake_covid_publisher.random_y = False
                fake_covid_publisher.location_y = param.value
                fake_covid_publisher.get_logger().info(f"y set to {param.value}")
        if param.name == "location_z":
                fake_covid_publisher.location_z = param.value
                fake_covid_publisher.get_logger().info(f"z set to {param.value}")

    return SetParametersResult(successful=True)


if __name__ == "__main__":
    main()
