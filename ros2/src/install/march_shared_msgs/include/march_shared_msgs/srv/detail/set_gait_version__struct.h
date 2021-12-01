// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:srv/SetGaitVersion.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__SET_GAIT_VERSION__STRUCT_H_
#define MARCH_SHARED_MSGS__SRV__DETAIL__SET_GAIT_VERSION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'gait'
// Member 'subgaits'
// Member 'versions'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetGaitVersion in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__SetGaitVersion_Request
{
  rosidl_runtime_c__String gait;
  rosidl_runtime_c__String__Sequence subgaits;
  rosidl_runtime_c__String__Sequence versions;
} march_shared_msgs__srv__SetGaitVersion_Request;

// Struct for a sequence of march_shared_msgs__srv__SetGaitVersion_Request.
typedef struct march_shared_msgs__srv__SetGaitVersion_Request__Sequence
{
  march_shared_msgs__srv__SetGaitVersion_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__SetGaitVersion_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetGaitVersion in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__SetGaitVersion_Response
{
  bool success;
  rosidl_runtime_c__String message;
} march_shared_msgs__srv__SetGaitVersion_Response;

// Struct for a sequence of march_shared_msgs__srv__SetGaitVersion_Response.
typedef struct march_shared_msgs__srv__SetGaitVersion_Response__Sequence
{
  march_shared_msgs__srv__SetGaitVersion_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__SetGaitVersion_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__SET_GAIT_VERSION__STRUCT_H_
