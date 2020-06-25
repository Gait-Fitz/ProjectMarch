from copy import deepcopy

from march_gait_selection.gait_selection import GaitSelection
from march_shared_classes.exceptions.gait_exceptions import GaitError, SubgaitNameNotFound, TransitionError
from march_shared_classes.gait.gait import Gait
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.gait.subgait import Subgait


class TransitionSubgait(Subgait):
    def __init__(self, joints, duration,
                 gait_type='walk_like', gait_name='Transition',
                 subgait_name='Transition_subgait', version='Default',
                 description='The subgait used to transition between two subgaits'):

        super(TransitionSubgait, self).__init__(joints, duration, gait_type, gait_name,
                                                subgait_name, version, description)

    @classmethod
    def from_subgait_names(cls, gait_selection, old_gait_name, new_gait_name, new_subgait_name, old_subgait_name=None):
        """Create a new transition subgait object between two given subgaits.

        :param gait_selection:
            The gait selection object which holds all the information about the gaits and subgaits
        :param old_gait_name:
            The name of the old (or current) gait
        :param new_gait_name:
            The name of the new gait which must be executed after the old gait
        :param new_subgait_name:
            Name of the subgait in which the transition will occur
        :param old_subgait_name:
            Possible to give an custom old subgait name, if not the same gait name will be used as new_subgait_name

        :return:
            A populated TransitionSubgait object which holds the data to transition between given gaits
        """
        old_subgait, new_subgait = cls._get_copy_of_subgaits(gait_selection, old_gait_name, new_gait_name,
                                                             new_subgait_name, old_subgait_name)

        transition_joints = cls._transition_joints(gait_selection.robot, old_subgait, new_subgait)
        transition_duration = new_subgait.duration

        transition_subgait = cls(transition_joints, transition_duration)

        cls._validate_transition_gait(old_subgait, transition_subgait, new_subgait)
        cls._validate_transition_trajectory(old_subgait, transition_subgait, new_subgait)

        return transition_subgait

    @staticmethod
    def _get_copy_of_subgaits(gait_selection, old_gait_name, new_gait_name, new_subgait_name, old_subgait_name):
        """Use the subgait names and the parsed subgaits in the gait selection object to get the subgait objects."""
        if not isinstance(gait_selection, GaitSelection):
            raise GaitError(msg='No valid gait selection module was parsed to the create the transition subgait')

        old_gait = gait_selection[old_gait_name]
        new_gait = gait_selection[new_gait_name]

        if old_gait is None:
            raise GaitError(msg='{gn} not found in parsed gait names from gait selection'
                            .format(gn=old_gait_name))

        if new_gait is None:
            raise GaitError(msg='{gn} not found in parsed gait names from gait selection'
                            .format(gn=new_gait_name))

        if old_gait.graph != new_gait.graph:
            raise GaitError(msg='Subgait graphs do not match between gait: {cg} and gait: {ng}'
                            .format(cg=old_gait_name, ng=new_gait_name))

        old_subgait_name = new_subgait_name if old_subgait_name is None else old_subgait_name

        old_subgait = deepcopy(old_gait[old_subgait_name])
        new_subgait = deepcopy(new_gait[new_subgait_name])

        if old_subgait is None:
            raise SubgaitNameNotFound(subgait_name=old_subgait_name)

        if new_subgait is None:
            raise SubgaitNameNotFound(subgait_name=new_subgait_name)

        return old_subgait, new_subgait

    @staticmethod
    def _transition_joints(robot, old_subgait, new_subgait):
        """Calculate a transition trajectory which starts at the old gait and ends with the endpoints of the new gait.

        :returns:
             list of joints which hold the transition setpoints including position, velocity and duration
        """
        max_duration = max(old_subgait.duration, new_subgait.duration)

        old_subgait.scale_timestamps_subgait(max_duration)
        new_subgait.scale_timestamps_subgait(max_duration)

        all_timestamps = old_subgait.get_unique_timestamps() + new_subgait.get_unique_timestamps()
        all_timestamps = sorted(set(all_timestamps))

        old_subgait.create_interpolated_setpoints(all_timestamps)
        new_subgait.create_interpolated_setpoints(all_timestamps)

        joints = []
        for old_joint in old_subgait.joints:

            joint_name = old_joint.name
            new_joint = new_subgait.get_joint(joint_name)
            limits = new_subgait.get_joint_limits_from_urdf(robot, joint_name)

            setpoints = []
            number_setpoints = len(new_subgait[0].setpoints)
            for transition_index in range(number_setpoints):
                factor = transition_index / (number_setpoints - 1.0)

                old_setpoint = old_joint[transition_index]
                new_setpoint = new_joint[transition_index]

                transition_setpoint = TransitionSubgait._transition_setpoint(old_setpoint, new_setpoint, factor)
                setpoints.append(transition_setpoint)

            joints.append(JointTrajectory(joint_name, limits, setpoints, old_joint.duration))

        return joints

    @staticmethod
    def _transition_setpoint(old_setpoint, new_setpoint, new_factor):
        """Create a transition setpoint with the use of the old setpoint, new setpoint and transition factor."""
        old_factor = 1.0 - new_factor

        position = (old_setpoint.position * old_factor) + (new_setpoint.position * new_factor)
        velocity = (old_setpoint.velocity * old_factor) + (new_setpoint.velocity * new_factor)

        return Setpoint(new_setpoint.time, position, velocity)

    @staticmethod
    def _validate_transition_gait(old_subgait, transition_subgait, new_subgait):
        """Validate the transition point by creating a gait object which checks the trajectories."""
        subgaits = {old_subgait.subgait_name: old_subgait,
                    transition_subgait.subgait_name: transition_subgait,
                    new_subgait.subgait_name: new_subgait}
        graph = {'start': old_subgait.subgait_name,
                 old_subgait.subgait_name: transition_subgait.subgait_name,
                 transition_subgait.subgait_name: new_subgait.subgait_name,
                 new_subgait.subgait_name: 'end'}

        try:
            Gait('transition', subgaits, graph)
        except Exception as error:
            TransitionError('Error when creating transition: {er}'.format(er=error))

    @staticmethod
    def _validate_transition_trajectory(old_subgait, transition_subgait, new_subgait):
        """Validate if the calculated trajectory is within the given subgaits."""
        for transition_joint in transition_subgait.joints:
            old_joint = old_subgait.get_joint(transition_joint.name)
            new_joint = new_subgait.get_joint(transition_joint.name)

            for old_setpoint, transition_setpoint, new_setpoint in zip(old_joint, transition_joint, new_joint):

                if old_setpoint.time != transition_setpoint.time:
                    raise TransitionError('The transition timestamp {tt} != the old timestamp {ot}'
                                          .format(tt=transition_setpoint.time, ot=old_setpoint.time))

                if new_setpoint.time != transition_setpoint.time:
                    raise TransitionError('The transition timestamp {tt} != the new timestamp {ot}'
                                          .format(tt=transition_setpoint.time, ot=new_setpoint.time))

                if old_setpoint.position < transition_setpoint.position:
                    if new_setpoint.position < transition_setpoint.position:
                        raise TransitionError('The transition position {tp} is not between the old {op} and new {np}'
                                              .format(tp=transition_setpoint.position, op=old_setpoint.position,
                                                      np=new_setpoint.position))

                if old_setpoint.position > transition_setpoint.position:
                    if new_setpoint.position > transition_setpoint.position:
                        raise TransitionError('The transition position {tp} is not between the new {np} and old {op}'
                                              .format(tp=transition_setpoint.position, op=old_setpoint.position,
                                                      np=new_setpoint.position))
