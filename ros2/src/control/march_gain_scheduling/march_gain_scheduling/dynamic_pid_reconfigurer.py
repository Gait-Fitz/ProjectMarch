from enum import Enum
from typing import List

from rclpy import time
from rclpy.node import Node

from march_shared_msgs.msg import CurrentGait, JointPIDs, PID

from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from std_srvs.srv._trigger import Trigger_Response, Trigger

from .one_step_linear_interpolation import interpolate


class GaitType(Enum):
    DEFAULT = 'default'
    WALK_LIKE = 'walk_like'
    SIT_LIKE = 'sit_like'
    STAIRS_LIKE = 'stairs_like'

    @classmethod
    def exists(cls, gait_type: str, node: Node):
        return gait_type is not None and gait_type != "" \
               and bool(node.get_parameters_by_prefix("gait_types.{gait_type}".format(gait_type=gait_type)))


class DynamicPIDReconfigurer:
    def __init__(self, joint_list: List[str], node: Node):
        node.get_logger().info("Started PID reconfigurer")
        self._joint_list = joint_list
        self._node = node
        self._gait_type = GaitType.DEFAULT
        self._timer = None

        _configuration = self._node.get_parameter("configuration").get_parameter_value().string_value

        self._srv = self._node.create_service(
            srv_type=Trigger,
            srv_name="/march/gain_scheduling/get_configuration",
            callback=lambda _, __: Trigger_Response(success=True,
                                                    message=_configuration)
        )

        self._sub = self._node.create_subscription(
            msg_type=CurrentGait,
            topic="/march/gait_selection/current_gait",
            callback=self.gait_selection_callback,
            qos_profile=DEFAULT_HISTORY_DEPTH)

        self._publisher = self._node.create_publisher(
            msg_type=JointPIDs,
            topic='/march/dynamic_reconfigure/PIDs',
            qos_profile=DEFAULT_HISTORY_DEPTH
        )

        self.current_gains = [List[int]] * len(joint_list)
        self.update_params(None, self.get_needed_gains(), False)
        self._node.get_logger().info(f"Exoskeleton was started with gain tuning for {_configuration}")

        # TODO: Implement this code to work with ros2 control:
        # self._params = [
        #     self._node.declare_parameter("/march/controller/trajectory/gains/" + joint)
        #     for joint in self._joint_list
        # ]

    def gait_selection_callback(self, msg: CurrentGait):
        """
        Callback function for if a gait_type message is sent over the '/march/gait_selection/current_gait' topic.
        This method checks if the new gait_type has its own PID values
        """
        new_gait_type = msg.gait_type
        self._node.get_logger().debug(f"Called with gait type: {new_gait_type}")
        if not GaitType.exists(new_gait_type, self._node):
            self._node.get_logger().warning(
                "The gait has unknown gait type of `{gait_type}`, default is set to walk_like".format(
                    gait_type=new_gait_type), once=True)
            self._node.get_logger().debug(
                "The gait has unknown gait type of `{gait_type}`, default is set to walk_like".format(
                    gait_type=new_gait_type))
            new_gait_type = GaitType.DEFAULT
        else:
            new_gait_type = GaitType(new_gait_type)

        if new_gait_type != self._gait_type:
            self._gait_type = new_gait_type
        else:
            self._node.get_logger().debug(
                f"The new gait type: {new_gait_type} equals this gait type: {self._gait_type}")
            return None

        needed_gains = self.get_needed_gains()
        # self.load_current_gains() # TODO: Implement this if we change to ros2 control

        if needed_gains != self.current_gains:
            self._node.get_logger().info(
                "Beginning PID interpolation for gait type: {0}".format(self._gait_type)
            )
            begin_time = self._node.get_clock().now()
            # To ensure there are no two timers running
            self._node.destroy_timer(self._timer)
            # Updates the PID params every 0.3 seconds with a callback to 'update_params'
            self._timer = self._node.create_timer(0.3, lambda: self.update_params(begin_time,
                                                                                  needed_gains,
                                                                                  not self._linearize))

    def update_params(self, begin_time, needed_gains, change_gradually):
        if self.current_gains != needed_gains:
            pid_messages = []
            for i in range(len(self._joint_list)):
                if change_gradually:
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
            self._node.get_logger().debug(
                f"Publishing gains:\nNeeded gains: {needed_gains}\nPublished gains: {self.current_gains}"
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

    def get_needed_gains(self, gait_type: GaitType = None) -> List[List[int]]:
        return [self.get_needed_pid(joint, gait_type) for joint in self._joint_list]

    def get_needed_pid(self, joint_name: str, gait_type: GaitType = None) -> List[int]:
        if gait_type is None:
            gait_type = self._gait_type
        return [
            self.get_joint_parameter_value(joint_name, 'p', gait_type),
            self.get_joint_parameter_value(joint_name, 'i', gait_type),
            self.get_joint_parameter_value(joint_name, 'd', gait_type),
        ]

    def get_joint_parameter_value(self, joint_name: str, param: str, gait_type: GaitType = GaitType.DEFAULT) -> int:
        """
        This method returns a parameter INTEGER value of the specified joint.
        If the parameter for the gait_type exists it returns that, otherwise it returns the default parameter values.
        This method is meant to retrieve the 'p','i' or 'd' value of a specific joint of the gait type.
        Example parameter that this method retrieves from this node is:
            get_joint_parameter_value(self, 'left_ankle', 'p', GaitType.SIT_LIKE) ->
                'gait_types.sit_like.left_ankle.p' and if that one doesn't exist: 'gait_types.default.left_ankle.p'

        To see these values check out the files in `../config/{robot}/{configuration}.yaml`.

        Args:
            joint_name (str): This is the name of the joint, should be in `self._joint_list`.
            param (str): This is the name of the parameter of the joint, e.g. 'p', 'i' or 'd'
            gait_type (GaitType, optional): This is the type of GaitType, if it isn't specified or the parameter
                for this gait_type doesn't exist it uses the GaitType.DEFAULT = 'default'.

        Returns
            int: The integer parameter value. (For pid parameters the return value is probably > 0)
        """
        param_name = "gait_types." + gait_type.value + "." + joint_name + '.' + param
        default_param_name = "gait_types." + GaitType.DEFAULT.value + "." + joint_name + '.' + param
        return self._node.get_parameter_or(param_name, self._node.get_parameter(default_param_name))\
            .get_parameter_value().integer_value

    # TODO: Refactor this if we change to ros2 control: (This is obsolete as long as we sent pid values over the bridge)
    # def load_current_gains(self):
    #     self.current_gains = []
    #     for joint_name in self._joint_list:
    #         gains = self._node.get_parameters_by_prefix("/march/controller/trajectory/gains/" + joint_name)
    #         gains[0].get_parameter_value().integer_value
    #         # gains = rospy.get_param("/march/controller/trajectory/gains/" + joint_name)
    #         self.current_gains.append([gains["p"].get_parameter_value().integer_value,
    #                                    gains["i"].get_parameter_value().integer_value,
    #                                    gains["d"].get_parameter_value().integer_value])

    @property
    def _gradient(self) -> float:
        return self._node.get_parameter("linear_slope").value

    @property
    def _linearize(self) -> bool:
        return self._node.get_parameter("linearize_gain_scheduling").get_parameter_value().bool_value
