"""Author: Marten Haitjema, MVII"""

import signal
import sys
import rclpy

from .gait_preprocessor import GaitPreprocessor
from rcl_interfaces.msg import SetParametersResult
from contextlib import suppress


def sys_exit(*_):
    sys.exit(0)


def main():
    rclpy.init()
    gait_preprocessor = GaitPreprocessor()

    gait_preprocessor.add_on_set_parameters_callback(
        lambda params: parameter_callback(gait_preprocessor, params)
    )

    signal.signal(signal.SIGTERM, sys_exit)

    with suppress(KeyboardInterrupt):
        rclpy.spin(gait_preprocessor)

    rclpy.shutdown()


def parameter_callback(gait_preprocessor, parameters: list) -> SetParametersResult:
    """Update parameter of gait_preprocessor and return if
    this is done succesfully.

    :param gait_preprocessor: gait_preprocessor class
    :type gait_preprocessor: GaitPreprocessor
    :param parameters: list containing the changed parameters
    :type parameters: list

    :returns: whether or not the parameters were set succesfully
    :rtype: SetParametersResult
    """
    for param in parameters:
        if param.name == "location_x":
            gait_preprocessor._location_x = param.value
        elif param.name == "location_y":
            gait_preprocessor._location_y = param.value
        if param.name == "simulate_points":
            gait_preprocessor._simulate_points = param.value
            gait_preprocessor.set_simulate_points_parameter()
        if param.name == "duration":
            gait_preprocessor._duration = param.value

        parameter_updated_logger(gait_preprocessor, param)

    return SetParametersResult(successful=True)


def parameter_updated_logger(gait_preprocessor, param):
    gait_preprocessor.logger.info(f"{param.name} set to {param.value}")