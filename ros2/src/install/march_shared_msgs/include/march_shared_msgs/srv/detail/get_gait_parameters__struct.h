// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:srv/GetGaitParameters.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__STRUCT_H_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STAIRS_UP'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__STAIRS_UP = 0
};

/// Constant 'STAIRS_DOWN'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__STAIRS_DOWN = 1
};

/// Constant 'RAMP_UP'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__RAMP_UP = 2
};

/// Constant 'RAMP_DOWN'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__RAMP_DOWN = 3
};

/// Constant 'SIT'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__SIT = 4
};

/// Constant 'CURB_UP'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__CURB_UP = 5
};

/// Constant 'CURB_DOWN'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__CURB_DOWN = 6
};

/// Constant 'CAMERA_FRONT'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__CAMERA_FRONT = 0
};

/// Constant 'CAMERA_BACK'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__CAMERA_BACK = 1
};

/// Constant 'TEST_CLOUD'.
enum
{
  march_shared_msgs__srv__GetGaitParameters_Request__TEST_CLOUD = 2
};

// Include directives for member types
// Member 'subgait_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/GetGaitParameters in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__GetGaitParameters_Request
{
  uint8_t realsense_category;
  uint8_t camera_to_use;
  rosidl_runtime_c__String subgait_name;
} march_shared_msgs__srv__GetGaitParameters_Request;

// Struct for a sequence of march_shared_msgs__srv__GetGaitParameters_Request.
typedef struct march_shared_msgs__srv__GetGaitParameters_Request__Sequence
{
  march_shared_msgs__srv__GetGaitParameters_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__GetGaitParameters_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'gait_parameters'
#include "march_shared_msgs/msg/detail/gait_parameters__struct.h"

// Struct defined in srv/GetGaitParameters in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__GetGaitParameters_Response
{
  bool success;
  rosidl_runtime_c__String error_message;
  march_shared_msgs__msg__GaitParameters gait_parameters;
} march_shared_msgs__srv__GetGaitParameters_Response;

// Struct for a sequence of march_shared_msgs__srv__GetGaitParameters_Response.
typedef struct march_shared_msgs__srv__GetGaitParameters_Response__Sequence
{
  march_shared_msgs__srv__GetGaitParameters_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__GetGaitParameters_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__STRUCT_H_
