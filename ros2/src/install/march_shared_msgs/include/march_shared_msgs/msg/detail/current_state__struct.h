// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/CurrentState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_STATE__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IDLE'.
enum
{
  march_shared_msgs__msg__CurrentState__IDLE = 0
};

/// Constant 'GAIT'.
enum
{
  march_shared_msgs__msg__CurrentState__GAIT = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'state'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/CurrentState in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__CurrentState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String state;
  int8_t state_type;
} march_shared_msgs__msg__CurrentState;

// Struct for a sequence of march_shared_msgs__msg__CurrentState.
typedef struct march_shared_msgs__msg__CurrentState__Sequence
{
  march_shared_msgs__msg__CurrentState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__CurrentState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_STATE__STRUCT_H_
