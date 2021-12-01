// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryGoal.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'trajectory'
#include "trajectory_msgs/msg/detail/joint_trajectory__struct.h"
// Member 'path_tolerance'
// Member 'goal_tolerance'
#include "control_msgs/msg/detail/joint_tolerance__struct.h"
// Member 'goal_time_tolerance'
#include "builtin_interfaces/msg/detail/duration__struct.h"

// Struct defined in msg/FollowJointTrajectoryGoal in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__FollowJointTrajectoryGoal
{
  trajectory_msgs__msg__JointTrajectory trajectory;
  control_msgs__msg__JointTolerance__Sequence path_tolerance;
  control_msgs__msg__JointTolerance__Sequence goal_tolerance;
  builtin_interfaces__msg__Duration goal_time_tolerance;
} march_shared_msgs__msg__FollowJointTrajectoryGoal;

// Struct for a sequence of march_shared_msgs__msg__FollowJointTrajectoryGoal.
typedef struct march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence
{
  march_shared_msgs__msg__FollowJointTrajectoryGoal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__STRUCT_H_
