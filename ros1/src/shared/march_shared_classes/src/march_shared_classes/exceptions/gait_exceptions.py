class GaitError(Exception):
    def __init__(self, msg=None):
        """Base class for exceptions in gait modules.

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "An error occurred with a gait module."
        super(GaitError, self).__init__(msg)


class GaitNameNotFoundError(GaitError):
    def __init__(self, gait_name, msg=None):
        """Class to raise an error when given gait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "Could not find gait name: {gait} in map.".format(gait=gait_name)

        super(GaitNameNotFoundError, self).__init__(msg)


class SubgaitNameNotFoundError(GaitError):
    def __init__(self, subgait_name, gait_name, msg=None):
        """Class to raise an error when given subgait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "Could not find subgait name {subgait} of gait {gait} in map.".format(
                subgait=subgait_name, gait=gait_name
            )

        super(SubgaitNameNotFoundError, self).__init__(msg)


class NonValidGaitContentError(GaitError):
    def __init__(self, gait_name=None, msg=None):
        """Class to raise an error when given gait has incorrect content .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "The given gait: {gn} has incorrect information".format(gn=gait_name)

        super(NonValidGaitContentError, self).__init__(msg)


class SubgaitGraphError(GaitError):
    def __init__(self, msg):
        super(SubgaitGraphError, self).__init__(msg)


class TransitionError(GaitError):
    def __init__(self, msg=None):
        """Class to raise an error when transition between two subgaits has an error .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "Subgaits can not transition"

        super(TransitionError, self).__init__(msg)


class SubgaitInterpolationError(GaitError):
    def __init__(self, msg=None):
        """Class to raise an error when it was not possible to interpolate between subgaits."""
        if msg is None:
            msg = "An error occurred while trying to merge two subgaits."

        super(SubgaitInterpolationError, self).__init__(msg)
