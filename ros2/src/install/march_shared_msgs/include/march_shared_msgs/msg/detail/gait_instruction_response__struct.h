// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/GaitInstructionResponse.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION_RESPONSE__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION_RESPONSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GAIT_REJECTED'.
enum
{
  march_shared_msgs__msg__GaitInstructionResponse__GAIT_REJECTED = -1
};

/// Constant 'GAIT_ACCEPTED'.
enum
{
  march_shared_msgs__msg__GaitInstructionResponse__GAIT_ACCEPTED = 0
};

/// Constant 'GAIT_FINISHED'.
enum
{
  march_shared_msgs__msg__GaitInstructionResponse__GAIT_FINISHED = 1
};

// Struct defined in msg/GaitInstructionResponse in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__GaitInstructionResponse
{
  int8_t result;
} march_shared_msgs__msg__GaitInstructionResponse;

// Struct for a sequence of march_shared_msgs__msg__GaitInstructionResponse.
typedef struct march_shared_msgs__msg__GaitInstructionResponse__Sequence
{
  march_shared_msgs__msg__GaitInstructionResponse * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__GaitInstructionResponse__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION_RESPONSE__STRUCT_H_
