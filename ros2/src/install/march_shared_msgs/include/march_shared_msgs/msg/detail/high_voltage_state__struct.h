// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/HighVoltageState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__HIGH_VOLTAGE_STATE__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__HIGH_VOLTAGE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/HighVoltageState in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__HighVoltageState
{
  float total_current;
  float hv1_current;
  float hv2_current;
  float hv3_current;
  float hv4_current;
} march_shared_msgs__msg__HighVoltageState;

// Struct for a sequence of march_shared_msgs__msg__HighVoltageState.
typedef struct march_shared_msgs__msg__HighVoltageState__Sequence
{
  march_shared_msgs__msg__HighVoltageState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__HighVoltageState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__HIGH_VOLTAGE_STATE__STRUCT_H_
