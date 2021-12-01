// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/MpcArray.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_ARRAY__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'array'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/MpcArray in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__MpcArray
{
  rosidl_runtime_c__double__Sequence array;
} march_shared_msgs__msg__MpcArray;

// Struct for a sequence of march_shared_msgs__msg__MpcArray.
typedef struct march_shared_msgs__msg__MpcArray__Sequence
{
  march_shared_msgs__msg__MpcArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__MpcArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_ARRAY__STRUCT_H_
