// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/CurrentIPDState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_IPD_STATE__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_IPD_STATE__STRUCT_H_

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
// Member 'menu_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/CurrentIPDState in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__CurrentIPDState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String menu_name;
} march_shared_msgs__msg__CurrentIPDState;

// Struct for a sequence of march_shared_msgs__msg__CurrentIPDState.
typedef struct march_shared_msgs__msg__CurrentIPDState__Sequence
{
  march_shared_msgs__msg__CurrentIPDState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__CurrentIPDState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_IPD_STATE__STRUCT_H_
