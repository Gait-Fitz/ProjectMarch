from time import sleep
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.executors import MultiThreadedExecutor
import threading


class LiveWidgetSubscriber(Node):
    def __init__(self, callback_function):
        super().__init__("live_widget_subscriber")
        self.listener_callback = callback_function
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            "/march/controller/trajectory/state",
            self.listener_callback,
            10,
        )