// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/CurrentGait.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_GAIT__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_GAIT__STRUCT_H_

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
// Member 'gait'
// Member 'subgait'
// Member 'version'
// Member 'gait_type'
#include "rosidl_runtime_c/string.h"
// Member 'duration'
#include "builtin_interfaces/msg/detail/duration__struct.h"

// Struct defined in msg/CurrentGait in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__CurrentGait
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String gait;
  rosidl_runtime_c__String subgait;
  rosidl_runtime_c__String version;
  builtin_interfaces__msg__Duration duration;
  rosidl_runtime_c__String gait_type;
} march_shared_msgs__msg__CurrentGait;

// Struct for a sequence of march_shared_msgs__msg__CurrentGait.
typedef struct march_shared_msgs__msg__CurrentGait__Sequence
{
  march_shared_msgs__msg__CurrentGait * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__CurrentGait__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_GAIT__STRUCT_H_
