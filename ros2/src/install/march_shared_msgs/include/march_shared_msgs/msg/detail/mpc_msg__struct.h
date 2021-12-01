// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/MpcMsg.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_MSG__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_MSG__STRUCT_H_

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
// Member 'diagnostics'
#include "march_shared_msgs/msg/detail/mpc_diagnostics__struct.h"
// Member 'joint'
#include "march_shared_msgs/msg/detail/mpc_joint__struct.h"

// Struct defined in msg/MpcMsg in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__MpcMsg
{
  std_msgs__msg__Header header;
  march_shared_msgs__msg__MpcDiagnostics diagnostics;
  march_shared_msgs__msg__MpcJoint__Sequence joint;
} march_shared_msgs__msg__MpcMsg;

// Struct for a sequence of march_shared_msgs__msg__MpcMsg.
typedef struct march_shared_msgs__msg__MpcMsg__Sequence
{
  march_shared_msgs__msg__MpcMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__MpcMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_MSG__STRUCT_H_
