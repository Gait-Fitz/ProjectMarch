// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryResult.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_RESULT__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_RESULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SUCCESSFUL'.
enum
{
  march_shared_msgs__msg__FollowJointTrajectoryResult__SUCCESSFUL = 0l
};

/// Constant 'INVALID_GOAL'.
enum
{
  march_shared_msgs__msg__FollowJointTrajectoryResult__INVALID_GOAL = -1l
};

/// Constant 'INVALID_JOINTS'.
enum
{
  march_shared_msgs__msg__FollowJointTrajectoryResult__INVALID_JOINTS = -2l
};

/// Constant 'OLD_HEADER_TIMESTAMP'.
enum
{
  march_shared_msgs__msg__FollowJointTrajectoryResult__OLD_HEADER_TIMESTAMP = -3l
};

/// Constant 'PATH_TOLERANCE_VIOLATED'.
enum
{
  march_shared_msgs__msg__FollowJointTrajectoryResult__PATH_TOLERANCE_VIOLATED = -4l
};

/// Constant 'GOAL_TOLERANCE_VIOLATED'.
enum
{
  march_shared_msgs__msg__FollowJointTrajectoryResult__GOAL_TOLERANCE_VIOLATED = -5l
};

// Include directives for member types
// Member 'error_string'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/FollowJointTrajectoryResult in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__FollowJointTrajectoryResult
{
  int32_t error_code;
  rosidl_runtime_c__String error_string;
} march_shared_msgs__msg__FollowJointTrajectoryResult;

// Struct for a sequence of march_shared_msgs__msg__FollowJointTrajectoryResult.
typedef struct march_shared_msgs__msg__FollowJointTrajectoryResult__Sequence
{
  march_shared_msgs__msg__FollowJointTrajectoryResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__FollowJointTrajectoryResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_RESULT__STRUCT_H_
