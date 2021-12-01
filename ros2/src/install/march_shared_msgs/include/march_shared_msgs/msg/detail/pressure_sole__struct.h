// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/PressureSole.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__STRUCT_H_

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
// Member 'pressure_soles_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'pressure_left'
// Member 'pressure_right'
// Member 'cop_left'
// Member 'cop_right'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/PressureSole in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__PressureSole
{
  std_msgs__msg__Header header;
  builtin_interfaces__msg__Time pressure_soles_time;
  float total_force_left;
  float total_force_right;
  rosidl_runtime_c__float__Sequence pressure_left;
  rosidl_runtime_c__float__Sequence pressure_right;
  rosidl_runtime_c__float__Sequence cop_left;
  rosidl_runtime_c__float__Sequence cop_right;
} march_shared_msgs__msg__PressureSole;

// Struct for a sequence of march_shared_msgs__msg__PressureSole.
typedef struct march_shared_msgs__msg__PressureSole__Sequence
{
  march_shared_msgs__msg__PressureSole * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__PressureSole__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__STRUCT_H_
