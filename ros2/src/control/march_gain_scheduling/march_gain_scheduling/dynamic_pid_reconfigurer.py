from enum import Enum

from rclpy import time
from rclpy.node import Node

from march_shared_msgs.msg import CurrentGait, JointPIDs, PID

from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from std_srvs.srv._trigger import Trigger_Response, Trigger

from .one_step_linear_interpolation import interpolate


class GaitType(Enum):
    WALK_LIKE = 'walk_like'
    SIT_LIKE = 'sit_like'
    STAIRS_LIKE = 'stairs_like'

    @classmethod
    def exists(cls, gait_type: str, node: Node):
        return gait_type is not None and gait_type != "" \
               and bool(node.get_parameters_by_prefix("gait_types.{gait_type}".format(gait_type=gait_type)))


DEFAULT_GAIT_TYPE = GaitType.WALK_LIKE


class DynamicPIDReconfigurer:
    def __init__(self, joint_list, node: Node):
        node.get_logger().info("Started PID reconfigurer")
        self._joint_list = joint_list
        self._node = node
        self._gait_type = DEFAULT_GAIT_TYPE
        self._timer = None
        # self._params = [
        #     self._node.declare_parameter("/march/controller/trajectory/gains/" + joint)
        #     for joint in self._joint_list
        # ]
        # self._node.set_parameters(self._params)  # Updates the params.

        self.current_gains = [self.get_needed_pid(i) for i in range(len(self._joint_list))]

        self._sub = self._node.create_subscription(
            msg_type=CurrentGait,
            topic="/march/gait_selection/current_gait",
            callback=self.gait_selection_callback,
            qos_profile=DEFAULT_HISTORY_DEPTH)

        _configuration = self._node.get_parameter("configuration").get_parameter_value().string_value

        self._srv = self._node.create_service(
            srv_type=Trigger,
            srv_name="/march/gain_scheduling/get_configuration",
            callback=lambda _, __: Trigger_Response(success=True,
                                                    message=_configuration)
        )

        self._publisher = self._node.create_publisher(
            msg_type=JointPIDs,
            topic='/march/dynamic_reconfigure/PIDs',
            qos_profile=DEFAULT_HISTORY_DEPTH
        )

        self._node.get_logger().info(f"Exoskeleton was started with gain tuning for {_configuration}")

    def gait_selection_callback(self, msg):
        new_gait_type = msg.gait_type
        self._node.get_logger().debug(f"Called with gait type: {new_gait_type}")
        if not GaitType.exists(new_gait_type, self._node):
            self._node.get_logger().warning(
                "The gait has unknown gait type of `{gait_type}`, default is set to walk_like".format(
                    gait_type=new_gait_type), once=True)
            new_gait_type = DEFAULT_GAIT_TYPE
        else:
            new_gait_type = GaitType(new_gait_type)

        if new_gait_type != self._gait_type:
            self._gait_type = new_gait_type
        else:
            self._node.get_logger().debug(
                f"The new gait type: {new_gait_type} equals this gait type: {self._gait_type}")
            return None

        needed_gains = self.get_needed_gains()
        # self.load_current_gains()

        if needed_gains != self.current_gains:
            self._node.get_logger().info(
                "Beginning PID interpolation for gait type: {0}".format(self._gait_type)
            )
            begin_time = self._node.get_clock().now()  # Changed clock to a ros2 clock check if this still works.
            # To ensure there are no two timers running
            self._node.destroy_timer(self._timer)
            # Updates the PID params every 0.3 seconds with a callback to 'update_params'
            self._timer = self._node.create_timer(0.3, lambda: self.update_params(begin_time, needed_gains))

    def update_params(self, begin_time, needed_gains):
        if self.current_gains != needed_gains:
            pid_messages = []
            _change_gradually = not self._linearize
            for i in range(len(self._joint_list)):
                if _change_gradually:
                    self.current_gains[i] = interpolate(
                        self.current_gains[i],
                        needed_gains[i],
                        self._gradient,
                        self._timer.time_since_last_call() / time.CONVERSION_CONSTANT,
                    )
                else:
                    self.current_gains[i] = needed_gains[i]
                # For the bridge. TODO: Change this to parameters for ros2 control
                pid_messages.append(
                    PID(p=self.current_gains[i][0],
                        i=self.current_gains[i][1],
                        d=self.current_gains[i][2])
                )
            self._node.get_logger().info(
                f"{self.current_gains == needed_gains}, -> Needed gains: {needed_gains}, but got: {self.current_gains}"
            )
            self._publisher.publish(JointPIDs(joints=pid_messages))
        else:
            self.stop_update_params_callback(begin_time)

    def stop_update_params_callback(self, begin_time):
        # This destroys the callback when it reached its end.
        if self._node.destroy_timer(self._timer):
            self._node.get_logger().info(
                "PID interpolation finished in {0}s".format(
                    self._node.get_clock().now() - begin_time
                )
            )
        else:
            self._node.get_logger().warning("The timer for dynamic pid reconfigure can't be destroyed")

    # def load_current_gains(self):
    #     self.current_gains = []
    #     for joint_name in self._joint_list:
    #         gains = self._node.get_parameters_by_prefix("/march/controller/trajectory/gains/" + joint_name)
    #         gains[0].get_parameter_value().integer_value
    #         # gains = rospy.get_param("/march/controller/trajectory/gains/" + joint_name)
    #         self.current_gains.append([gains["p"].get_parameter_value().integer_value,
    #                                    gains["i"].get_parameter_value().integer_value,
    #                                    gains["d"].get_parameter_value().integer_value])

    # Method that pulls the PID values from the gains_per_gait_type.yaml config file
    # Rethink this method, if the only use is in this class. Return result is always wrangled back in a for loop.

    def get_needed_gains(self, gait_type: GaitType = None):
        return [self.get_needed_pid(i, gait_type) for i in range(len(self._joint_list))]

    def get_needed_pid(self, joint_index, gait_type: GaitType = None):
        if gait_type is None:
            gait_type = self._gait_type
        name_prefix = "gait_types." + gait_type.value + "." + self._joint_list[joint_index]
        self._node.get_logger().info(
            "Started lookup table with prefix {0}".format(name_prefix)
        )
        return [
            self._node.get_parameter(name_prefix + ".p").get_parameter_value().integer_value,
            self._node.get_parameter(name_prefix + ".i").get_parameter_value().integer_value,
            self._node.get_parameter(name_prefix + ".d").get_parameter_value().integer_value
        ]

    @property
    def _gradient(self) -> float:
        return self._node.get_parameter("linear_slope").value

    @property
    def _linearize(self) -> bool:
        return self._node.get_parameter("linearize_gain_scheduling").get_parameter_value().bool_value
