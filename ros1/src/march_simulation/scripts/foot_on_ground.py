#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Bool

EPSILON = 0.00001

class FootOnGround(object):
    def __init__(self):
        rospy.loginfo("Initializing foot on ground node")
        self.link_sub = rospy.Subscriber('/gazebo/link_states', LinkStates,
                                         callback=self.update_foot_on_ground, queue_size=2)
        self.left_foot_pub = rospy.Publisher('/march/left_foot_on_ground', Bool)
        # self.right_foot_pub = rospy.Publisher('/march/right_foot_on_ground', Bool)

    def update_foot_on_ground(self, msg: LinkStates) -> None:
        for (index, name) in msg.name:
            rospy.loginfo(f"{index} --- {name}")
            if "march::ankle_plate_left" == name:
                if msg.pose[index]['y'] < EPSILON:
                    self.left_foot_pub.publish(Bool(True))
                else:
                    self.left_foot_pub.publish(Bool(False))


def main():
    node = FootOnGround()
    rospy.init_node("foot_on_ground")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
