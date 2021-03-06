from dynamic_reconfigure.client import Client
import rospy

from march_shared_msgs.msg import CurrentGait
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from .one_step_linear_interpolation import interpolate


class DynamicPIDReconfigurer:
    def __init__(self, joint_list):
        self._gait_type = None
        self._joint_list = joint_list
        self._last_update_times = []
        self._clients = [
            Client("/march/controller/trajectory/gains/" + joint, timeout=90)
            for joint in self._joint_list
        ]
        self.current_gains = []
        rospy.Subscriber(
            "/march/gait_selection/current_gait",
            CurrentGait,
            callback=self.gait_selection_callback,
        )
        rospy.Service(
            "/march/gain_scheduling/get_configuration",
            Trigger,
            handler=self.configuration_cb,
        )
        self._linearize = rospy.get_param("~linearize_gain_scheduling")
        self._gradient = rospy.get_param("~linear_slope")
        self._configuration = rospy.get_param("~configuration")
        rospy.loginfo(
            f"Exoskeleton was started with gain tuning for " f"{self._configuration}"
        )

    def gait_selection_callback(self, msg):
        new_gait_type = msg.gait_type
        if (
            new_gait_type is None
            or new_gait_type == ""
            or not rospy.has_param(
                "~gait_types/{gait_type}".format(gait_type=new_gait_type)
            )
        ):
            rospy.logwarn_once(
                "The gait has unknown gait type of `{gait_type}`, default is set to walk_like".format(
                    gait_type=new_gait_type
                )
            )
            new_gait_type = "walk_like"

        self._gait_type = new_gait_type
        self.load_current_gains()
        needed_gains = [self.look_up_table(i) for i in range(len(self._joint_list))]

        if not self.done_interpolation_test(needed_gains):
            rate = rospy.Rate(10)

            rospy.loginfo(
                "Beginning PID interpolation for gait type: {0}".format(self._gait_type)
            )
            begin_time = rospy.get_time()
            self._last_update_times = len(self._joint_list) * [begin_time]
            while not self.done_interpolation_test(needed_gains):
                self.client_update(needed_gains)
                rate.sleep()
            rospy.loginfo(
                "PID interpolation finished in {0}s".format(
                    rospy.get_time() - begin_time
                )
            )

    def client_update(self, needed_gains):
        for i in range(len(self._joint_list)):
            if self._linearize:
                current_time = rospy.get_time()
                time_interval = current_time - self._last_update_times[i]

                self.current_gains[i] = interpolate(
                    self.current_gains[i],
                    needed_gains[i],
                    self._gradient,
                    time_interval,
                )

                self._last_update_times[i] = rospy.get_time()
            else:
                self.current_gains[i] = needed_gains[i]

            self._clients[i].update_configuration(
                {
                    "p": self.current_gains[i][0],
                    "i": self.current_gains[i][1],
                    "d": self.current_gains[i][2],
                }
            )

    def load_current_gains(self):
        self.current_gains = []
        for joint_name in self._joint_list:
            gains = rospy.get_param("/march/controller/trajectory/gains/" + joint_name)
            self.current_gains.append([gains["p"], gains["i"], gains["d"]])

    # Method that pulls the PID values from the gains_per_gait_type.yaml config file
    def look_up_table(self, joint_index):
        gains = rospy.get_param(
            "~gait_types/" + self._gait_type + "/" + self._joint_list[joint_index],
            {"p": None, "i": None, "d": None},
        )
        return [gains["p"], gains["i"], gains["d"]]

    def done_interpolation_test(self, needed_gains):
        return all(
            self.current_gains[i] == needed_gains[i]
            for i in range(len(self._joint_list))
        )

    def configuration_cb(self, req: TriggerRequest):
        return TriggerResponse(success=True, message=self._configuration)
