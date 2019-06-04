import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from Joint import Joint

from march_shared_resources.msg import Setpoint
from march_rqt_gait_generator.UserInterfaceController import notify


class Gait:

    def __init__(self, joints, duration,
                 name="Walk", subgait="right_open", version="First try", description="Just a simple gait"):
        self.joints = joints
        self.name = name
        self.subgait = subgait
        self.version = version
        self.description = description
        self.duration = duration
        self.current_time = 0

    def to_joint_trajectory(self):
        joint_trajectory = JointTrajectory()

        timestamps = self.get_unique_timestamps()

        for joint in self.joints:
            joint_trajectory.joint_names.append(joint.name)

        for timestamp in timestamps:
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = rospy.Duration(timestamp)
            for joint in self.joints:
                interpolated_setpoint = joint.get_interpolated_setpoint(timestamp)

                if interpolated_setpoint.time != timestamp:
                    rospy.logerr("Time mismatch in joint " + joint.name + " at timestamp " + timestamp)
                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)
            joint_trajectory.points.append(joint_trajectory_point)

        return joint_trajectory

    def to_setpoints(self):
        user_defined_setpoints = []
        timestamps = self.get_unique_timestamps()
        for timestamp in timestamps:
            user_defined_setpoint = Setpoint()
            user_defined_setpoint.time_from_start = rospy.Duration.from_sec(timestamp)
            for joint in self.joints:
                for setpoint in joint.setpoints:
                    if setpoint.time == timestamp:
                        user_defined_setpoint.joint_names.append(joint.name)
            user_defined_setpoints.append(user_defined_setpoint)
        return user_defined_setpoints

    def get_unique_timestamps(self):
        timestamps = []
        for joint in self.joints:
            for setpoint in joint.setpoints:
                timestamps.append(setpoint.time)

        return sorted(set(timestamps))

    def get_joint(self, name):
        for i in range(0, len(self.joints)):
            if self.joints[i].name == name:
                return self.joints[i]
        rospy.logerr("Joint with name " + name + " does not exist in gait " + self.name + ".")
        return None

    def has_multiple_setpoints_before_duration(self, duration):
        for joint in self.joints:
            count = 0
            for setpoint in joint.setpoints:
                if setpoint.time <= duration:
                    count += 1
            if count < 2:
                return False
        return True

    def has_setpoints_after_duration(self, duration):
        for joint in self.joints:
            for setpoint in joint.setpoints:
                if setpoint.time > duration:
                    return True
        return False

    # Setters to allow changing values in a callback
    def set_name(self, name):
        self.name = name

    def set_description(self, description):
        self.description = description

    def set_version(self, version):
        self.version = version

    def set_subgait(self, subgait):
        self.subgait = subgait

    def set_duration(self, duration, rescale=False):
        for joint in self.joints:
            # Loop in reverse to avoid out of bounds errors while deleting.
            for setpoint in reversed(joint.setpoints):
                if rescale:
                    setpoint.time = duration * setpoint.time / self.duration
                else:
                    if setpoint.time > duration:
                        joint.setpoints.remove(setpoint)
            joint.interpolated_setpoints = joint.interpolate_setpoints()

        self.duration = duration

    def set_current_time(self, current_time):
        self.current_time = current_time

    def get_mirrored(self, key_1, key_2):
        if key_1 in self.subgait:
            mirrored_subgait_name = self.subgait.replace(key_1, key_2)
        elif key_2 in self.subgait:
            mirrored_subgait_name = self.subgait.replace(key_2, key_1)
        else:
            notify("Could not mirror Subgait.",
                   "Subgait name " + self.subgait + " does not contain required key " + key_1)
            return False

        mirrored_joints = []
        for joint in self.joints:
            if key_1 in joint.name:
                mirrored_name = joint.name.replace(key_1, key_2)
            elif key_2 in joint.name:
                mirrored_name = joint.name.replace(key_2, key_1)
            else:
                continue

            mirrored_joint = Joint(mirrored_name, joint.limits, joint.setpoints, joint.duration)
            mirrored_joints.append(mirrored_joint)

        return Gait(mirrored_joints, self.duration, self.name, mirrored_subgait_name, self.version, self.description)

