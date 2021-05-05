# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2020 Bas Volkers
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# Version 3 as published by the Free Software Foundation WITH
# additional terms published by Project MARCH per section 7 of
# the GNU General Public License Version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License INCLUDING the additional terms for
# more details.
#
# You should have received a copy of the GNU General Public License
# AND the additional terms along with this program. If not,
# see <https://projectmarch.nl/s/LICENSE> and
# <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.

#!/usr/bin/env python

import unittest

from parameterized import parameterized
import rospy

from march_shared_msgs.srv import (
    GetParamBool,
    GetParamBoolRequest,
    GetParamFloat,
    GetParamFloatRequest,
    GetParamInt,
    GetParamIntRequest,
    GetParamString,
    GetParamStringList,
    GetParamStringListRequest,
    GetParamStringRequest,
    SetParamBool,
    SetParamBoolRequest,
    SetParamFloat,
    SetParamFloatRequest,
    SetParamInt,
    SetParamIntRequest,
    SetParamString,
    SetParamStringList,
    SetParamStringListRequest,
    SetParamStringRequest,
)


PKG = "march_parameter_server"
NAME = "test_parameter_server"


class TestParameterServer(unittest.TestCase):
    @parameterized.expand(
        [
            ["float", GetParamFloat, GetParamFloatRequest, 4.5],
            ["int", GetParamInt, GetParamIntRequest, 9],
            ["string", GetParamString, GetParamStringRequest, "test_string123"],
            ["bool", GetParamBool, GetParamBoolRequest, True],
            ["string_list", GetParamStringList, GetParamStringListRequest, ["abcdef"]],
        ]
    )
    def test_get_parameter(self, param_type, service_type, service_request_type, value):
        rospy.set_param("/test/{param_type}_get".format(param_type=param_type), value)

        rospy.wait_for_service(
            "/march/parameter_server/get_param_{param_type}".format(
                param_type=param_type
            )
        )

        service = rospy.ServiceProxy(
            "/march/parameter_server/get_param_{param_type}".format(
                param_type=param_type
            ),
            service_type,
        )
        response = service.call(
            service_request_type("/test/{param_type}_get".format(param_type=param_type))
        )

        self.assertEqual(response.value, value)
        self.assertTrue(response.success)

    @parameterized.expand(
        [
            ["float", SetParamFloat, SetParamFloatRequest, -7.2],
            ["int", SetParamInt, SetParamIntRequest, -3],
            ["string", SetParamString, SetParamStringRequest, "abcdef"],
            ["bool", SetParamBool, SetParamBoolRequest, False],
            [
                "string_list",
                SetParamStringList,
                SetParamStringListRequest,
                ["abcde", "fghijklm"],
            ],
        ]
    )
    def test_set_parameter(self, param_type, service_type, service_request_type, value):
        rospy.wait_for_service(
            "/march/parameter_server/set_param_{param_type}".format(
                param_type=param_type
            )
        )

        service = rospy.ServiceProxy(
            "/march/parameter_server/set_param_{param_type}".format(
                param_type=param_type
            ),
            service_type,
        )
        response = service.call(
            service_request_type(
                "/test/{param_type}_set".format(param_type=param_type), value
            )
        )

        self.assertEqual(
            rospy.get_param("/test/{param_type}_set".format(param_type=param_type)),
            value,
        )
        self.assertTrue(response.success)

    @parameterized.expand(
        [
            ["float", GetParamFloat, GetParamFloatRequest, 0.0],
            ["int", GetParamInt, GetParamIntRequest, 0],
            ["string", GetParamString, GetParamStringRequest, ""],
            ["bool", GetParamBool, GetParamBoolRequest, False],
            ["string_list", GetParamStringList, GetParamStringListRequest, []],
        ]
    )
    def test_get_parameter_not_existing(
        self, param_type, service_type, service_request_type, default_value
    ):
        rospy.wait_for_service(
            "/march/parameter_server/get_param_{param_type}".format(
                param_type=param_type
            )
        )

        service = rospy.ServiceProxy(
            "/march/parameter_server/get_param_{param_type}".format(
                param_type=param_type
            ),
            service_type,
        )
        response = service.call(
            service_request_type(
                "/not_existing_param_{param_type}".format(param_type=param_type)
            )
        )

        self.assertEqual(response.value, default_value)
        self.assertFalse(response.success)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, NAME, TestParameterServer)
