// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/AfterLimitJointCommand.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__AFTER_LIMIT_JOINT_COMMAND__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__AFTER_LIMIT_JOINT_COMMAND__STRUCT_H_

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
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'position_command'
// Member 'effort_command'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/AfterLimitJointCommand in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__AfterLimitJointCommand
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__double__Sequence position_command;
  rosidl_runtime_c__double__Sequence effort_command;
} march_shared_msgs__msg__AfterLimitJointCommand;

// Struct for a sequence of march_shared_msgs__msg__AfterLimitJointCommand.
typedef struct march_shared_msgs__msg__AfterLimitJointCommand__Sequence
{
  march_shared_msgs__msg__AfterLimitJointCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__AfterLimitJointCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__AFTER_LIMIT_JOINT_COMMAND__STRUCT_H_
