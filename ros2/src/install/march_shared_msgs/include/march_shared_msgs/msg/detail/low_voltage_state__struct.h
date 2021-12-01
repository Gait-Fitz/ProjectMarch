// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/LowVoltageState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__LOW_VOLTAGE_STATE__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__LOW_VOLTAGE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/LowVoltageState in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__LowVoltageState
{
  float lv1_current;
  float lv2_current;
  uint32_t lv1_ok;
  uint32_t lv2_ok;
} march_shared_msgs__msg__LowVoltageState;

// Struct for a sequence of march_shared_msgs__msg__LowVoltageState.
typedef struct march_shared_msgs__msg__LowVoltageState__Sequence
{
  march_shared_msgs__msg__LowVoltageState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__LowVoltageState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__LOW_VOLTAGE_STATE__STRUCT_H_
