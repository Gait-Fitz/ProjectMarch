"""Author: George Vegelien, MVII."""
import rospy
from dynamic_reconfigure.client import Client

from march_shared_msgs.msg import JointPIDs


class DynamicPIDReconfigure:
    """Sets the new parameters for the gains in the ros1 parameter server.

    This class exists because we use ros1 control which is dependent on the parameter values for the pid values,
    which should be stored in the ros1 parameter server.

    Because we are in the middle of porting from ros1 to ros2. We have code that handles the logic behind the new PID
    values in ros2, but we still need to change it for control in ros1. So that is why this class is created.

    Todo:
        * Remove this class if we switch to ros2 control.

    Args:
        joint_list (List[str]): String list of al joint names in alphabetical order.

    Attributes:
        _clients (dynamic_reconfigure.Client): All PID parameters for all joints,
            wrapped in an client object to be able to change the values.
    """

    def __init__(self, joint_list):
        self._clients = [Client("/march/controller/trajectory/gains/" + joint, timeout=90) for joint in joint_list]
        rospy.Subscriber(
            "/march/dynamic_reconfigure/PIDs",
            JointPIDs,
            callback=self.update_pid,
        )

    def update_pid(self, msg: JointPIDs) -> None:
        """Updates the ros1 parameter server with the new PID value in the JointPIDs message."""
        for i, pid_msg in enumerate(msg.joints):
            self._clients[i].update_configuration({"p": pid_msg.p, "i": pid_msg.i, "d": pid_msg.d})


def main():
    """Starts the "march_dynamic_reconfigure_node" and node and runs the DynamicPIDReconfigure object from it."""
    rospy.init_node("march_dynamic_reconfigure_node")

    while not rospy.is_shutdown() and not rospy.has_param("/march/joint_names"):
        rospy.sleep(0.5)
        rospy.logdebug("Waiting on /march/joint_names to be available")

    if rospy.is_shutdown():
        return

    joint_list = rospy.get_param("/march/joint_names")
    DynamicPIDReconfigure(joint_list)

    rospy.spin()


if __name__ == "__main__":
    main()
