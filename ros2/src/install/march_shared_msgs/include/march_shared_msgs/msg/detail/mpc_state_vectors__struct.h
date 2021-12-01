// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/MpcStateVectors.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_STATE_VECTORS__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_STATE_VECTORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'states'
// Member 'inputs'
#include "march_shared_msgs/msg/detail/mpc_array__struct.h"

// Struct defined in msg/MpcStateVectors in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__MpcStateVectors
{
  march_shared_msgs__msg__MpcArray__Sequence states;
  march_shared_msgs__msg__MpcArray__Sequence inputs;
} march_shared_msgs__msg__MpcStateVectors;

// Struct for a sequence of march_shared_msgs__msg__MpcStateVectors.
typedef struct march_shared_msgs__msg__MpcStateVectors__Sequence
{
  march_shared_msgs__msg__MpcStateVectors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__MpcStateVectors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_STATE_VECTORS__STRUCT_H_
