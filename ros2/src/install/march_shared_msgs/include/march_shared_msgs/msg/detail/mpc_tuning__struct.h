// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/MpcTuning.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_TUNING__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_TUNING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'q_weights'
// Member 'r_weights'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/MpcTuning in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__MpcTuning
{
  rosidl_runtime_c__float__Sequence q_weights;
  rosidl_runtime_c__float__Sequence r_weights;
  int32_t horizon;
} march_shared_msgs__msg__MpcTuning;

// Struct for a sequence of march_shared_msgs__msg__MpcTuning.
typedef struct march_shared_msgs__msg__MpcTuning__Sequence
{
  march_shared_msgs__msg__MpcTuning * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__MpcTuning__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_TUNING__STRUCT_H_
