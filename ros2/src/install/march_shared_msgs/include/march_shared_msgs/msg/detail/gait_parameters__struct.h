// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/GaitParameters.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/GaitParameters in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__GaitParameters
{
  double first_parameter;
  double second_parameter;
  double side_step_parameter;
} march_shared_msgs__msg__GaitParameters;

// Struct for a sequence of march_shared_msgs__msg__GaitParameters.
typedef struct march_shared_msgs__msg__GaitParameters__Sequence
{
  march_shared_msgs__msg__GaitParameters * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__GaitParameters__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__STRUCT_H_
