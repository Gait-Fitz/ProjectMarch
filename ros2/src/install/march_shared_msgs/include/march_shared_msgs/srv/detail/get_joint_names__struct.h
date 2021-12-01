// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:srv/GetJointNames.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_JOINT_NAMES__STRUCT_H_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_JOINT_NAMES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/GetJointNames in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__GetJointNames_Request
{
  uint8_t structure_needs_at_least_one_member;
} march_shared_msgs__srv__GetJointNames_Request;

// Struct for a sequence of march_shared_msgs__srv__GetJointNames_Request.
typedef struct march_shared_msgs__srv__GetJointNames_Request__Sequence
{
  march_shared_msgs__srv__GetJointNames_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__GetJointNames_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'joint_names'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/GetJointNames in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__GetJointNames_Response
{
  rosidl_runtime_c__String__Sequence joint_names;
} march_shared_msgs__srv__GetJointNames_Response;

// Struct for a sequence of march_shared_msgs__srv__GetJointNames_Response.
typedef struct march_shared_msgs__srv__GetJointNames_Response__Sequence
{
  march_shared_msgs__srv__GetJointNames_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__GetJointNames_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_JOINT_NAMES__STRUCT_H_
