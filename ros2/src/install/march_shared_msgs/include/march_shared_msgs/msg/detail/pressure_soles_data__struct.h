// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/PressureSolesData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__STRUCT_H_

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
// Member 'left_foot'
// Member 'right_foot'
#include "march_shared_msgs/msg/detail/pressure_sole_data__struct.h"

// Struct defined in msg/PressureSolesData in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__PressureSolesData
{
  std_msgs__msg__Header header;
  march_shared_msgs__msg__PressureSoleData left_foot;
  march_shared_msgs__msg__PressureSoleData right_foot;
} march_shared_msgs__msg__PressureSolesData;

// Struct for a sequence of march_shared_msgs__msg__PressureSolesData.
typedef struct march_shared_msgs__msg__PressureSolesData__Sequence
{
  march_shared_msgs__msg__PressureSolesData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__PressureSolesData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__STRUCT_H_
