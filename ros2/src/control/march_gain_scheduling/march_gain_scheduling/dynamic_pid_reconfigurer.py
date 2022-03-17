from enum import Enum
from typing import List, final, Optional

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.time import Time
from rclpy.timer import Timer

from march_shared_msgs.msg import CurrentGait, JointPIDs, PID

from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from std_srvs.srv._trigger import Trigger_Response, Trigger

from .one_step_linear_interpolation import interpolate


class GaitType(Enum):
    """The types of gaits there are.

    These should be the same as specified in the `config/{robot}/{configuration}.yaml files.
    """
    DEFAULT = 'default'
    WALK_LIKE = 'walk_like'
    SIT_LIKE = 'sit_like'
    STAIRS_LIKE = 'stairs_like'

    @classmethod
    def exists(cls, gait_type: str, node: Node):
        """Checks if the gait_type actually exists.

        Args:
            gait_type (str, Optional): The string_name of the gait_type, to check if it exists.
                If None or empty this method will always return False.
            node (Node): The node for which it will check if it has the gait_types.{gait_type} parameter.

        Returns:
            bool: True if:
                * the gait_type is either one of the enums.
                * if the node has a parameter value for the gait_type.
        """
        return (gait_type is not None and gait_type != "") and \
               (gait_type in GaitType._value2member_map_ or
                bool(node.get_parameters_by_prefix("gait_types.{gait_type}"
                                                   .format(gait_type=gait_type))))


class DynamicPIDReconfigurer:
    """Handles setting dynamic PID values for joints within different gait types.

    This object is connected with a node from where it logs and retrieves the parameters.
    The node parameters are:
        configuration (str): Which yaml file used in loading the different PID values.
            The yaml files used can be found at 'config/{robot}/{configuration}.yaml'
        linear_slope (float): The value for how fast the old PID converges to the new PID value
            if they are different between gait types. The higher the value the faster it goes.
        linearize_gain_scheduling (bool):  Boolean for if we want to slowly converge between PID values
            or set them instantly. `True` for converging, and `False` for instant.
        gait_types.default.{joint_name}.{[p, i, d]} (int):
            The PID value of the joint for the default gait type.
        gait_types.{GaitType}.{joint_name}.{[p, i, d]} (int, Optional):
            The PID value of the joint for a certain gait type.

    Args:
        joint_list (List[str]): String list of al joint names in alphabetical order.
        node (Node): The node from where it logs, publishes, retrieves parameters from and creates services from.

    Attributes:
        _joint_list (List[str]): String list of al joint names in alphabetical order.
        _node (Node): The node from where it logs, publishes, retrieves parameters from and creates services from.
        _gait_type (GaitType): The current gait_type the exo uses.
        _timer (rclpy.Timer): Timer object that calls the `update_params(...)` method every
            {TIME_BETWEEN_GRADUAL_UPDATES} seconds, to make the PID update gradual.
        _srv (rclpy.Service): To retrieve the .yaml configuration file used to declare the PID values
            for every joint with certain gait types. Mainly used by the rqt notetaker.
        _sub (rclpy.Subscription): To listen if the current_gait type is changed.
        _publisher (rclpy.Publisher): To set the PID parameter values over the bridge.
        _current_gains (List[List[float]]): A list of `int` lists of the 'p', 'i', and 'd' values for all joints,
                what the current PID values are.
    """
    TIME_BETWEEN_GRADUAL_UPDATES: final = 0.3

    def __init__(self, joint_list: List[str], node: Node):
        node.get_logger().info("Started PID reconfigurer")
        self._joint_list = joint_list
        self._node = node
        self._gait_type = GaitType.DEFAULT
        self._timer: Optional[Timer] = None

        _configuration = self._node.get_parameter("configuration").get_parameter_value().string_value

        self._srv: Service = self._node.create_service(
            srv_type=Trigger,
            srv_name="/march/gain_scheduling/get_configuration",
            callback=lambda _, __: Trigger_Response(success=True,
                                                    message=_configuration)
        )

        self._sub: Subscription = self._node.create_subscription(
            msg_type=CurrentGait,
            topic="/march/gait_selection/current_gait",
            callback=self.gait_selection_callback,
            qos_profile=DEFAULT_HISTORY_DEPTH)

        # TODO: Remove if we change to ros 2 control.
        self._publisher: Publisher = self._node.create_publisher(
            msg_type=JointPIDs,
            topic='/march/dynamic_reconfigure/PIDs',
            qos_profile=DEFAULT_HISTORY_DEPTH
        )

        self._current_gains: List[List[float]] = [[0.0]] * len(joint_list)
        self.update_params(self.get_needed_gains())
        self._node.get_logger().info(f"Exoskeleton was started with gain tuning for {_configuration}")

    def gait_selection_callback(self, msg: CurrentGait) -> None:
        """Callback function for if a gait_type message is sent over the '/march/gait_selection/current_gait' topic.

        This method checks if the new PID values for the new gait_type are different from the current PID values.
        If the PID values are different they are updated with the `update_params(...)` method.
        This can be done either gradually or instant depending on the value for the node parameter
        `linearize_gain_scheduling` (which can be changed at runtime).

        If the new gait_type doesn't exist it uses GaitType.DEFAULT.

        TODO:
            * Implement a load current gains from the parameters if we change to ros2 control.

        Args:
            msg (march_shared_msgs.msg.CurrentGait): The message that should trigger this callback.
                Within the message only the string value for `msg.gait_type` is used.
        """
        new_gait_type = msg.gait_type
        self._node.get_logger().debug(f"Called with gait type: {new_gait_type}")
        if not GaitType.exists(new_gait_type, self._node):
            if new_gait_type is not None or new_gait_type != "":  # Logging for if the gait type isn't known.
                self._node.get_logger().warning(
                    "The gait has unknown gait type of `{gait_type}`, default is set to walk_like".format(
                        gait_type=new_gait_type), once=True)
                self._node.get_logger().debug(
                    "The gait has unknown gait type of `{gait_type}`, default is set to walk_like".format(
                        gait_type=new_gait_type))
            new_gait_type = GaitType.DEFAULT
        else:
            new_gait_type = GaitType(new_gait_type)

        if new_gait_type == self._gait_type:
            self._node.get_logger().debug(
                f"The new gait type: {new_gait_type} equals this gait type: {self._gait_type}")
            return
        else:
            self._gait_type = new_gait_type

        needed_gains = self.get_needed_gains()
        # self.load_current_gains() # TODO: Implement this if we change to ros2 control

        if needed_gains != self._current_gains:
            self._node.get_logger().info(
                "Beginning PID interpolation for gait type: {0}".format(self._gait_type)
            )
            begin_time = self._node.get_clock().now()
            # To ensure there are no two timers running
            self._node.destroy_timer(self._timer)
            # Updates the PID params every 0.3 seconds with a callback to 'update_params'
            self._timer = self._node.create_timer(self.TIME_BETWEEN_GRADUAL_UPDATES,
                                                  lambda: self.update_params(needed_gains,
                                                                             begin_time,
                                                                             self._linearize))

    def update_params(self, needed_gains: List[List[float]],
                      begin_time: Time = None,
                      change_gradually: bool = False) -> None:
        """Function to update the pid parameters.

        This should be used as callback function within a timer to update the parameters every x amount of seconds.
        To stop this callback, call `stop_update_params_callback(...)`,
        or destroy the timer object associated with it.

        TODO:
            * When control is ported to ROS2 it should update the params directly instead of sending them over a topic.

        Note:
            The PID updates happens every {self.TIME_BETWEEN_GRADUAL_UPDATES} and updates it with
                {self.TIME_BETWEEN_GRADUAL_UPDATES} * {self._gradiant}.

        Args:
            needed_gains (List[List[float]]): A list of `int` lists of the 'p', 'i', and 'd' values for all joints,
                where the actual PID values should be updated to.
            begin_time (rclpy.Time, optional): Gives the start time of when the callback is started.
                This is only used for printing the interpolation time when it is finished.
            change_gradually (bool, optional): True if you want to slowly update your pid to your new needed gains,
                false if you want to set them instantly.
                If you set this value to True you should also set the `begin_time`.
        """
        if self._current_gains != needed_gains:
            pid_messages = []
            for i in range(len(self._joint_list)):
                if change_gradually:
                    self._current_gains[i] = interpolate(
                        self._current_gains[i],
                        needed_gains[i],
                        self._gradient,
                        self.TIME_BETWEEN_GRADUAL_UPDATES,
                    )
                else:
                    self._current_gains[i] = needed_gains[i]
                # For the bridge. TODO: Change this to parameters for ros2 control
                pid_messages.append(
                    PID(p=self._current_gains[i][0],
                        i=self._current_gains[i][1],
                        d=self._current_gains[i][2])
                )
            self._node.get_logger().debug(
                f"Publishing gains:\nNeeded gains: {needed_gains}\nPublished gains: {self._current_gains}\n"
            )
            self._publisher.publish(JointPIDs(joints=pid_messages))
        else:
            self.stop_update_params_callback(begin_time)

    def stop_update_params_callback(self, begin_time: Time) -> None:
        """This method destroys the current timer object and stops the callback when it reached its end.

        Args:
            begin_time (rclpy.Time): Gives the start time of when the callback is started.
                This is only used for printing the interpolation time.
        """
        if self._node.destroy_timer(self._timer):
            self._node.get_logger().info(
                "PID interpolation finished in {0}".format(
                    (self._node.get_clock().now() - begin_time).to_msg()
                )
            )
        else:
            self._node.get_logger().warning("The timer for dynamic pid reconfigure can't be destroyed")

    def get_needed_gains(self, gait_type: GaitType = None) -> List[List[float]]:
        """This method returns all the pid integer values of the gait type.

        If the gait type is not specified it uses the current set object gait_type.

        Args:
            gait_type (GaitType, optional): This is the type of GaitType,
                if it isn't specified the current object gait_type is used.
                if the gait type doesn't exist it uses the GaitType.DEFAULT = 'default'.

        Returns
            List[List[float]]: A list of `float` lists of size 3,
                where the first value is the p, second the i, and third the d.
        """
        return [self.get_needed_pid(joint, gait_type) for joint in self._joint_list]

    def get_needed_pid(self, joint_name: str, gait_type: GaitType = None) -> List[float]:
        """This method returns a list of the yaml specified 3 pid values for the selected joint name and gait type.

        Args:
            joint_name (str): This is the name of the joint, should be in `self._joint_list`.
            gait_type (GaitType, optional): This is the type of GaitType,
                if it isn't specified the current object gait_type is used.
                if the gait type doesn't exist it uses the GaitType.DEFAULT = 'default'.

        Returns
            List[float]: A `float` list of size 3, where the first value is the p, second the i, and third the d.
        """
        if gait_type is None:
            gait_type = self._gait_type
        return [
            self.get_joint_parameter_value(joint_name, 'p', gait_type),
            self.get_joint_parameter_value(joint_name, 'i', gait_type),
            self.get_joint_parameter_value(joint_name, 'd', gait_type),
        ]

    def get_joint_parameter_value(self, joint_name: str, param: str, gait_type: GaitType = GaitType.DEFAULT) -> float:
        """This method returns a parameter INTEGER value of the specified joint.

        If the parameter for the gait_type exists it returns that, otherwise it returns the default parameter values.
        This method is meant to retrieve the 'p','i' or 'd' value of a specific joint of the gait type.
        Example parameter that this method retrieves from this node is:
            get_joint_parameter_value(self, 'left_ankle', 'p', GaitType.SIT_LIKE) ->
                'gait_types.sit_like.left_ankle.p' and if that one doesn't exist: 'gait_types.default.left_ankle.p'

        To see these values check out the files in `../config/{robot}/{configuration}.yaml`.

        TODO:
            * If this method is to slow, add memoization.

        Args:
            joint_name (str): This is the name of the joint, should be in `self._joint_list`.
            param (str): This is the name of the parameter of the joint, e.g. 'p', 'i' or 'd'
            gait_type (GaitType, optional): This is the type of GaitType, if it isn't specified or the parameter
                for this gait_type doesn't exist it uses the GaitType.DEFAULT = 'default'.

        Returns
            float: The integer parameter value. (For pid parameters the return value is probably > 0)
        """
        param_name = "gait_types." + gait_type.value + "." + joint_name + '.' + param
        default_param_name = "gait_types." + GaitType.DEFAULT.value + "." + joint_name + '.' + param
        return float(self._node.get_parameter_or(param_name, self._node.get_parameter(default_param_name)).value)

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
