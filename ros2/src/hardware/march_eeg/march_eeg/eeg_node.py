"""Author: George Vegelien, MVII."""
import sys

import rclpy
import std_msgs.msg
from rclpy.node import Node
from std_msgs.msg import Header

from march_eeg.eeg import Eeg

from march_utility.utilities.duration import NSEC_IN_SEC
from march_shared_msgs.msg import GaitInstruction

EEG_DEFAULT_WALK_GAIT = "dynamic_walk"
EEG_GAIT_NAME = "eeg"
UPDATE_SPEED = 8 / NSEC_IN_SEC


class EEGNode(Node):
    NODE_NAME_ID = "MARCH EEG Node"

    def __init__(self, action_gait_name):
        super().__init__("march_eeg", automatically_declare_parameters_from_overrides=True)
        self._logger.info("Created node 'march_eeg'")
        self.active = False
        self._instruction_gait_pub = self.create_publisher(
            msg_type=GaitInstruction,
            topic="/march/input_device/instruction",
            qos_profile=1,
        )

        self.create_subscription(
            msg_type=std_msgs.msg.Bool,
            topic="/march/eeg/on_off",
            qos_profile=10,
            callback=self.callback_activate_toggle
        )
        self.gait_name = action_gait_name
        headset_name = self.get_parameter("headset").get_parameter_value().string_value
        file_name = self.get_parameter("file_name").get_parameter_value().string_value
        self.eeg = Eeg(headset_name, file_name, self._logger)
        self.previous_walking_thought = False
        self.update_timer = self.create_timer(UPDATE_SPEED, self.update)

    def update(self) -> None:
        if self.active:
            current_walking_thought = self.eeg.walking_thought
            if self.previous_walking_thought != current_walking_thought:
                if current_walking_thought:
                    self.publish_start_walking()
                else:
                    self.publish_stop_walking()

    def publish_start_walking(self) -> None:
        """Publish on `/march/input_device/instruction` to start walking."""
        self._logger.info("Start walking")
        self.previous_walking_thought = True
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._clock.now().to_msg()),
                type=GaitInstruction.GAIT,
                gait_name=self.gait_name,
                id=self.NODE_NAME_ID
            )
        )

    def publish_stop_walking(self) -> None:
        """Publish a message on `/march/input_device/instruction` to stop the gait."""
        self._logger.info("Stop walking")
        self.previous_walking_thought = False
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._clock.now().to_msg()),
                type=GaitInstruction.STOP,
                gait_name="",
                id=self.NODE_NAME_ID
            )
        )

    def callback_activate_toggle(self, msg: std_msgs.msg.Bool):
        if msg.data:
            self._logger.info("---------------------------------------------- Here ON!!")
            if not self.active:
                self.eeg.start()
                self.active = True
        else:
            self._logger.info("---------------------------------------------- Here OFFFFFFFFFFFFF")
            self.eeg.stop()
            self.active = False
            self.publish_stop_walking()


def main():
    """Starts the "march_gain_scheduling_node" node and passes it to DynamicPIDReconfigurer."""
    rclpy.init(args=sys.argv)
    node = EEGNode(EEG_DEFAULT_WALK_GAIT)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
