"""Author: Control, MIV; George Vegelien, MVII."""
from typing import List


def interpolate(current_gains: List[float], needed_gains: List[float], gradient: float, delta_t: float) -> List[float]:
    """Performs linear interpolation between the current_gains and the needed_gains.

    The multiplication of gradient * delta_t is how fast current_gains reaches needed_gains.

    Args:
        current_gains (List[float]): The current gain values, and thus the starting value of the interpolation.
        needed_gains (List[float]): The needed gain values, thus the value that we wish to interpolate to.
        gradient (float): The gradient for how fast it reaches the needed_gains.
        delta_t (float): The time since last interpolation.

    Returns:
        List[float]: A list with all the new gain values. All values in this list should be either
        between the values of current_gains and needed_gains, or equal to the values of needed_gains.

    Raises:
        UnequalLengthError: If the length of `current_gains` isn't the same as the length of `needed_gains`.
        NegativeValueError: If either the gradient or the delta_t has a negative value.
    """
    if len(current_gains) != len(needed_gains):
        raise UnequalLengthError(
            "current_gains and needed_gains do not have the same length, "
            "\ncurrent_gains = {current_gains}\nneeded_gains = {needed_gains}".format(
                current_gains=current_gains, needed_gains=needed_gains
            )
        )
    if gradient <= 0 or delta_t <= 0:
        raise NegativeValueError(
            "gradient or delta_t are below zero, gradiant: {gradient}, delta_t: {delta_t}".format(
                gradient=gradient, delta_t=delta_t
            )
        )
    next_gains = [0.0] * len(current_gains)
    for i in range(len(current_gains)):
        if current_gains[i] > needed_gains[i]:
            next_gains[i] = max(needed_gains[i], current_gains[i] - (gradient * delta_t))
        else:
            next_gains[i] = min(needed_gains[i], current_gains[i] + (gradient * delta_t))
    return next_gains


class UnequalLengthError(Exception):
    """Unequal length error, for if the gains aren't of the same size."""


class NegativeValueError(Exception):
    """Negative value error, for if a positive value is expected."""
