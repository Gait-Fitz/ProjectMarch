// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:srv/PublishTestDataset.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__PUBLISH_TEST_DATASET__STRUCT_H_
#define MARCH_SHARED_MSGS__SRV__DETAIL__PUBLISH_TEST_DATASET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NEXT'.
enum
{
  march_shared_msgs__srv__PublishTestDataset_Request__NEXT = 0
};

/// Constant 'STOP'.
enum
{
  march_shared_msgs__srv__PublishTestDataset_Request__STOP = 1
};

/// Constant 'CUSTOM'.
enum
{
  march_shared_msgs__srv__PublishTestDataset_Request__CUSTOM = 2
};

/// Constant 'SAVE'.
enum
{
  march_shared_msgs__srv__PublishTestDataset_Request__SAVE = 3
};

// Include directives for member types
// Member 'pointcloud_file_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/PublishTestDataset in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__PublishTestDataset_Request
{
  uint8_t selected_mode;
  rosidl_runtime_c__String pointcloud_file_name;
  bool save_camera_back;
} march_shared_msgs__srv__PublishTestDataset_Request;

// Struct for a sequence of march_shared_msgs__srv__PublishTestDataset_Request.
typedef struct march_shared_msgs__srv__PublishTestDataset_Request__Sequence
{
  march_shared_msgs__srv__PublishTestDataset_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__PublishTestDataset_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/PublishTestDataset in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__PublishTestDataset_Response
{
  bool success;
  rosidl_runtime_c__String message;
} march_shared_msgs__srv__PublishTestDataset_Response;

// Struct for a sequence of march_shared_msgs__srv__PublishTestDataset_Response.
typedef struct march_shared_msgs__srv__PublishTestDataset_Response__Sequence
{
  march_shared_msgs__srv__PublishTestDataset_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__PublishTestDataset_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__PUBLISH_TEST_DATASET__STRUCT_H_
