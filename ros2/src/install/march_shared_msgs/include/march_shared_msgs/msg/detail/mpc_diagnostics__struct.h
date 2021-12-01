// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/MpcDiagnostics.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_DIAGNOSTICS__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_DIAGNOSTICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MpcDiagnostics in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__MpcDiagnostics
{
  double preparation_time;
  double feedback_time;
  double total_time;
  int32_t preparation_status;
  int32_t feedback_status;
  float cost;
} march_shared_msgs__msg__MpcDiagnostics;

// Struct for a sequence of march_shared_msgs__msg__MpcDiagnostics.
typedef struct march_shared_msgs__msg__MpcDiagnostics__Sequence
{
  march_shared_msgs__msg__MpcDiagnostics * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__MpcDiagnostics__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_DIAGNOSTICS__STRUCT_H_
