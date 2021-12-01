// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:srv/GetMoveItTrajectory.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__STRUCT_H_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'swing_leg'
#include "rosidl_runtime_c/string.h"
// Member 'swing_leg_target_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'stance_leg_target'
#include "sensor_msgs/msg/detail/joint_state__struct.h"

// Struct defined in srv/GetMoveItTrajectory in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__GetMoveItTrajectory_Request
{
  rosidl_runtime_c__String swing_leg;
  geometry_msgs__msg__Pose swing_leg_target_pose;
  sensor_msgs__msg__JointState stance_leg_target;
} march_shared_msgs__srv__GetMoveItTrajectory_Request;

// Struct for a sequence of march_shared_msgs__srv__GetMoveItTrajectory_Request.
typedef struct march_shared_msgs__srv__GetMoveItTrajectory_Request__Sequence
{
  march_shared_msgs__srv__GetMoveItTrajectory_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__GetMoveItTrajectory_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'trajectory'
#include "trajectory_msgs/msg/detail/joint_trajectory__struct.h"

// Struct defined in srv/GetMoveItTrajectory in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__GetMoveItTrajectory_Response
{
  bool success;
  trajectory_msgs__msg__JointTrajectory trajectory;
} march_shared_msgs__srv__GetMoveItTrajectory_Response;

// Struct for a sequence of march_shared_msgs__srv__GetMoveItTrajectory_Response.
typedef struct march_shared_msgs__srv__GetMoveItTrajectory_Response__Sequence
{
  march_shared_msgs__srv__GetMoveItTrajectory_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__GetMoveItTrajectory_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__STRUCT_H_
