#[[
 Copyright (C) 2021 Bas Volkers, Katja Schmahl, Thijs Veen,
                    Wolf Nederpel
 Copyright (C) 2020 Bas Volkers, Olav de Haas, Rutger van Beek,
                    Thijs Raymakers, Wolf Nederpel
 Copyright (C) 2019 Olav de Haas
 Copyright (C) 2018 Isha Dijks

 This program is free software: you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 Version 3 as published by the Free Software Foundation WITH
 additional terms published by Project MARCH per section 7 of
 the GNU General Public License Version 3.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License INCLUDING the additional terms for
 more details.

 You should have received a copy of the GNU General Public License
 AND the additional terms along with this program. If not,
 see <https://projectmarch.nl/s/LICENSE> and
 <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.
]]

cmake_minimum_required(VERSION 3.0.2)
project(march_shared_msgs)

find_package(catkin REQUIRED COMPONENTS
    control_msgs
    geometry_msgs
    message_generation
    std_msgs
    trajectory_msgs
    sensor_msgs
)

add_message_files(
    FILES
    AfterLimitJointCommand.msg
    Alive.msg
    CurrentGait.msg
    CurrentState.msg
    Error.msg
    GaitInstruction.msg
    GaitInstructionResponse.msg
    GaitParameters.msg
    HighVoltageNet.msg
    MotorControllerState.msg
    JointValues.msg
    LowVoltageNet.msg
    PowerDistributionBoardState.msg
    PressureSole.msg
    PressureSoleData.msg
    PressureSolesData.msg
)
add_message_files(
    DIRECTORY
    msg/mpc_msgs
    FILES
    MpcArray.msg
    MpcStateVectors.msg
    MpcTuning.msg
    MpcDiagnostics.msg
    MpcJoint.msg
    MpcMsg.msg
)

add_service_files(
    FILES
    CapturePointPose.srv
    ChangeComLevel.srv
    ContainsGait.srv
    GetGaitParameters.srv
    GetMoveItTrajectory.srv
    GetParamBool.srv
    GetParamFloat.srv
    GetParamInt.srv
    GetParamString.srv
    GetParamStringList.srv
    GetPossibleComLevels.srv
    PossibleGaits.srv
    SetGaitVersion.srv
    SetObstacleSizeRampLike.srv
    SetObstacleSizeBlockLike.srv
    SetParamBool.srv
    SetParamFloat.srv
    SetParamInt.srv
    SetParamString.srv
    SetParamStringList.srv
)

generate_messages(
    DEPENDENCIES
    control_msgs
    std_msgs
    trajectory_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    message_runtime
    std_msgs
    trajectory_msgs
    sensor_msgs
)
