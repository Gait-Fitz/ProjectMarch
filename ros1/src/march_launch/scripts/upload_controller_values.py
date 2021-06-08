#! /usr/bin/env python
import rospy
from urdf_parser_py import urdf


def get_params_for_actuation(joint):
    return [
        f"/march/controller/trajectory/gains/{joint}",
        f"/march/controller/trajectory/constraints/{joint}",
    ]


def main():
    rospy.init_node("upload_controller_values")
    robot = urdf.Robot.from_parameter_server("/robot_description")
    actuating_joint_names = []
    fixed_joint_names = []
    for joint in robot.joints:
        if joint.type != "fixed":
            actuating_joint_names.append(joint.name)
        else:
            fixed_joint_names.append(joint.name)
    rospy.set_param("/march/joint_names", actuating_joint_names)
    rospy.set_param("/march/controller/trajectory/joints", actuating_joint_names)

    for joint in fixed_joint_names:
        for param in get_params_for_actuation(joint):
            try:
                rospy.delete_param(param)
            except KeyError:
                continue


if __name__ == "__main__":
    main()
