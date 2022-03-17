from typing import List


def interpolate(current_gains: List[float], needed_gains: List[float], gradient: float, delta_t: float) -> List[float]:
    """Performs linear interpolation between the current_gains and the needed_gains.

    The multiplication of gradiant * delta_t is how fast current_gains reaches needed_gains.

    Args:
        current_gains (List[int]): The current gain values from where it should start the interpolation.
        needed_gains (List[int]): The needed gain values where we wish to interpolate to.
        gradient (float): The gradiant for how fast it reaches the needed_gains.
        delta_t (float): The time since last interpolation.


    Returns:
        List[int]: A list with al the new gain values. All values in this list should be either,
        between the values of current_gains and needed_gains, or equal to the values of needed_gains.

    Raises:
        UnequalLengthError: If the length of `current_gains` isn't the same as the length of `needed_gains`.
        NegativeValueError: If either the gradient or the delta_t has a negative value.
    """
    if len(current_gains) != len(needed_gains):
        raise UnequalLengthError(
            "current_gains and needed_gains do not have the same length, "
            "\ncurrent_gains = {current_gains}\nneeded_gains = {needed_gains}".format(current_gains=current_gains,
                                                                                      needed_gains=needed_gains)
        )
    if gradient <= 0 or delta_t < 0:
        raise NegativeValueError(
            "gradient or delta_t are below zero, gradiant: {gradient}, delta_t: {delta_t}".format(gradient=gradient,
                                                                                                  delta_t=delta_t))
    next_gains = [0.0] * len(current_gains)
    for i in range(len(current_gains)):
        if current_gains[i] > needed_gains[i]:
            next_gains[i] = max(needed_gains[i], current_gains[i] - (gradient * delta_t))
        else:
            next_gains[i] = min(needed_gains[i], current_gains[i] + (gradient * delta_t))
    return next_gains


class UnequalLengthError(Exception):
    pass


class NegativeValueError(Exception):
    pass
