// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/Alive.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__ALIVE__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__ALIVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'id'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/Alive in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__Alive
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String id;
} march_shared_msgs__msg__Alive;

// Struct for a sequence of march_shared_msgs__msg__Alive.
typedef struct march_shared_msgs__msg__Alive__Sequence
{
  march_shared_msgs__msg__Alive * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__Alive__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__ALIVE__STRUCT_H_
