import rospy
from dynamic_reconfigure.client import Client

from march_shared_msgs.msg import JointPIDs


class DynamicPIDReconfigure:
    def __init__(self, joint_list):
        self._clients = [
            Client("/march/controller/trajectory/gains/" + joint, timeout=90)
            for joint in joint_list
        ]
        rospy.Subscriber(
            "/march/dynamic_reconfigure/PIDs",
            JointPIDs,
            callback=self.update_pid,
        )

    def update_pid(self, msg: JointPIDs):
        for i, pid_msg in enumerate(msg.joints):
            self._clients[i].update_configuration(
                {'p': pid_msg.p,
                 'i': pid_msg.i,
                 'd': pid_msg.d}
            )


def main():
    rospy.init_node("march_dynamic_reconfigure_node")

    while not rospy.is_shutdown() and not rospy.has_param("/march/joint_names"):
        rospy.sleep(0.5)
        rospy.logdebug("Waiting on /march/joint_names to be available")

    if rospy.is_shutdown():
        return

    joint_list = rospy.get_param("/march/joint_names")
    DynamicPIDReconfigure(joint_list)

    rospy.spin()


if __name__ == '__main__':
    main()
