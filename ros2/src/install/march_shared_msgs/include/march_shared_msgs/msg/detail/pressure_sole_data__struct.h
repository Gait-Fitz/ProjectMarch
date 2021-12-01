// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/PressureSoleData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'side'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/PressureSoleData in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__PressureSoleData
{
  rosidl_runtime_c__String side;
  float heel_right;
  float heel_left;
  float met1;
  float hallux;
  float met3;
  float toes;
  float met5;
  float arch;
} march_shared_msgs__msg__PressureSoleData;

// Struct for a sequence of march_shared_msgs__msg__PressureSoleData.
typedef struct march_shared_msgs__msg__PressureSoleData__Sequence
{
  march_shared_msgs__msg__PressureSoleData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__PressureSoleData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__STRUCT_H_
