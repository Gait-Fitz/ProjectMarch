import copy

from numpy_ringbuffer import RingBuffer
import rospy

from march_shared_classes.gait.joint_trajectory import JointTrajectory

from .modifiable_setpoint import ModifiableSetpoint


class ModifiableJointTrajectory(JointTrajectory):
    setpoint_class = ModifiableSetpoint

    def __init__(self, name, limits, setpoints, duration, gait_generator=None):
        self.setpoints_history = RingBuffer(capacity=100, dtype=list)
        self.setpoints_redo_list = RingBuffer(capacity=100, dtype=list)
        self.gait_generator = gait_generator

        super(ModifiableJointTrajectory, self).__init__(name, limits, setpoints, duration)
        self.interpolated_setpoints = self.interpolate_setpoints()

    @classmethod
    def from_dict(cls, subgait_dict, joint_name, limits, duration, gait_generator):
        user_defined_setpoints = subgait_dict.get('setpoints')
        if user_defined_setpoints:
            joint_trajectory_dict = subgait_dict['trajectory']
            setpoints = []
            for actual_setpoint in user_defined_setpoints:
                if joint_name in actual_setpoint['joint_names']:
                    setpoints.append(cls._get_setpoint_at_duration(
                        joint_trajectory_dict, joint_name, actual_setpoint['time_from_start']))
            if setpoints[0].time != 0:
                rospy.logwarn('First setpoint of {0} has been set '
                              'from {1} to 0'.format(joint_name, setpoints[0].time))
            if setpoints[-1].time != duration:
                rospy.logwarn('Last setpoint of {0} has been set '
                              'from {1} to {2}'.format(joint_name, setpoints[0].time, duration))
            return cls(joint_name,
                       limits,
                       setpoints,
                       duration,
                       gait_generator,
                       )

        rospy.logwarn('This subgait has no user defined setpoints.')
        return super(ModifiableJointTrajectory, cls).from_dict(subgait_dict, joint_name, limits,
                                                               duration, gait_generator)

    @staticmethod
    def _get_setpoint_at_duration(joint_trajectory_dict, joint_name, duration):
        for point in joint_trajectory_dict['points']:
            if point['time_from_start'] == duration:
                index = joint_trajectory_dict['joint_names'].index(joint_name)
                time = rospy.Duration(point['time_from_start']['secs'], point['time_from_start']['nsecs']).to_sec()

                return ModifiableSetpoint(time, point['positions'][index], point['velocities'][index])
        return None

    def set_setpoints(self, setpoints):
        self.setpoints = setpoints
        self.enforce_limits()
        self.interpolated_setpoints = self.interpolate_setpoints()

    @property
    def duration(self):
        return self._duration

    @duration.setter
    def duration(self, duration):
        self._duration = duration
        self.enforce_limits()

    def get_interpolated_position(self, time):
        for i in range(0, len(self.interpolated_setpoints[0])):
            if self.interpolated_setpoints[0][i] > time:
                return self.interpolated_setpoints[1][i]

        return self.interpolated_setpoints[1][-1]

    def enforce_limits(self):
        self.setpoints[0].time = 0
        self.setpoints[-1].time = self.duration

        for setpoint in self.setpoints:
            setpoint.position = min(max(setpoint.position,
                                        self.limits.lower),
                                    self.limits.upper)
            setpoint.velocity = min(max(setpoint.velocity,
                                        -self.limits.velocity),
                                    self.limits.velocity)

    def add_interpolated_setpoint(self, time):
        self.add_setpoint(self.get_interpolated_setpoint(time))

    def add_setpoint(self, setpoint):
        self.save_setpoints()
        # Calculate at what index the new setpoint should be added.
        new_index = len(self.setpoints)
        for i in range(0, len(self.setpoints)):
            if self.setpoints[i].time > setpoint.time:
                new_index = i
                break

        rospy.logdebug('adding setpoint {0} at index {1}'.format(setpoint, new_index))
        self.setpoints.insert(new_index, setpoint)

        self.enforce_limits()
        self.interpolated_setpoints = self.interpolate_setpoints()

    def remove_setpoint(self, index):
        self.save_setpoints()
        del self.setpoints[index]
        self.enforce_limits()
        self.interpolated_setpoints = self.interpolate_setpoints()

    def save_setpoints(self, single_joint_change=True):
        self.setpoints_history.append(copy.deepcopy(self.setpoints))    # list(...) to copy instead of pointer
        if single_joint_change:
            self.gait_generator.save_changed_joints([self])

    def invert(self):
        self.save_setpoints(single_joint_change=False)
        self.setpoints = list(reversed(self.setpoints))
        for setpoint in self.setpoints:
            setpoint.invert(self.duration)
        self.interpolated_setpoints = self.interpolate_setpoints()

    def undo(self):
        self.setpoints_redo_list.append(list(self.setpoints))
        self.setpoints = self.setpoints_history.pop()
        self.interpolated_setpoints = self.interpolate_setpoints()

    def redo(self):
        self.setpoints_history.append(list(self.setpoints))
        self.setpoints = self.setpoints_redo_list.pop()
        self.interpolated_setpoints = self.interpolate_setpoints()
