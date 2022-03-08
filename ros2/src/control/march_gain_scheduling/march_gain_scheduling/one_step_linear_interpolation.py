from typing import List


def interpolate(current_gains, needed_gains, gradient, delta_t) -> List[int]:
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
    next_gains = [0] * len(current_gains)
    for i in range(len(current_gains)):
        if current_gains[i] > needed_gains[i]:
            next_gains[i] = max(needed_gains[i], current_gains[i] - (int(gradient * delta_t) + 1))
        else:
            next_gains[i] = min(needed_gains[i], current_gains[i] + (int(gradient * delta_t) + 1))
    return next_gains


class UnequalLengthError(Exception):
    pass


class NegativeValueError(Exception):
    pass
