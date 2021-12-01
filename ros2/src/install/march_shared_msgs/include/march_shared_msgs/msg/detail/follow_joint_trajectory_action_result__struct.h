// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionResult.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'status'
#include "actionlib_msgs/msg/detail/goal_status__struct.h"
// Member 'result'
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_result__struct.h"

// Struct defined in msg/FollowJointTrajectoryActionResult in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__FollowJointTrajectoryActionResult
{
  std_msgs__msg__Header header;
  actionlib_msgs__msg__GoalStatus status;
  march_shared_msgs__msg__FollowJointTrajectoryResult result;
} march_shared_msgs__msg__FollowJointTrajectoryActionResult;

// Struct for a sequence of march_shared_msgs__msg__FollowJointTrajectoryActionResult.
typedef struct march_shared_msgs__msg__FollowJointTrajectoryActionResult__Sequence
{
  march_shared_msgs__msg__FollowJointTrajectoryActionResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__FollowJointTrajectoryActionResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__STRUCT_H_
