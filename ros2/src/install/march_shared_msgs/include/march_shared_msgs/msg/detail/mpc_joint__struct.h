// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/MpcJoint.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'estimation'
// Member 'reference'
#include "march_shared_msgs/msg/detail/mpc_state_vectors__struct.h"
// Member 'tuning'
#include "march_shared_msgs/msg/detail/mpc_tuning__struct.h"

// Struct defined in msg/MpcJoint in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__MpcJoint
{
  march_shared_msgs__msg__MpcStateVectors estimation;
  march_shared_msgs__msg__MpcStateVectors reference;
  march_shared_msgs__msg__MpcTuning tuning;
} march_shared_msgs__msg__MpcJoint;

// Struct for a sequence of march_shared_msgs__msg__MpcJoint.
typedef struct march_shared_msgs__msg__MpcJoint__Sequence
{
  march_shared_msgs__msg__MpcJoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__MpcJoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__STRUCT_H_
