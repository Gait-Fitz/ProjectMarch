// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/GaitInstruction.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'UNKNOWN'.
enum
{
  march_shared_msgs__msg__GaitInstruction__UNKNOWN = -1
};

/// Constant 'STOP'.
enum
{
  march_shared_msgs__msg__GaitInstruction__STOP = 0
};

/// Constant 'GAIT'.
enum
{
  march_shared_msgs__msg__GaitInstruction__GAIT = 1
};

/// Constant 'PAUSE'.
enum
{
  march_shared_msgs__msg__GaitInstruction__PAUSE = 2
};

/// Constant 'CONTINUE'.
enum
{
  march_shared_msgs__msg__GaitInstruction__CONTINUE = 3
};

/// Constant 'INCREMENT_STEP_SIZE'.
enum
{
  march_shared_msgs__msg__GaitInstruction__INCREMENT_STEP_SIZE = 4
};

/// Constant 'DECREMENT_STEP_SIZE'.
enum
{
  march_shared_msgs__msg__GaitInstruction__DECREMENT_STEP_SIZE = 5
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'gait_name'
// Member 'id'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/GaitInstruction in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__GaitInstruction
{
  std_msgs__msg__Header header;
  int8_t type;
  rosidl_runtime_c__String gait_name;
  rosidl_runtime_c__String id;
} march_shared_msgs__msg__GaitInstruction;

// Struct for a sequence of march_shared_msgs__msg__GaitInstruction.
typedef struct march_shared_msgs__msg__GaitInstruction__Sequence
{
  march_shared_msgs__msg__GaitInstruction * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__GaitInstruction__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION__STRUCT_H_
