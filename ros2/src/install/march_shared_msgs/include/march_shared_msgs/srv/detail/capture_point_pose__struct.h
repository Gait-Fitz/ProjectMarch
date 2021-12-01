// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:srv/CapturePointPose.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__STRUCT_H_
#define MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/CapturePointPose in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__CapturePointPose_Request
{
  double duration;
} march_shared_msgs__srv__CapturePointPose_Request;

// Struct for a sequence of march_shared_msgs__srv__CapturePointPose_Request.
typedef struct march_shared_msgs__srv__CapturePointPose_Request__Sequence
{
  march_shared_msgs__srv__CapturePointPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__CapturePointPose_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'capture_point'
#include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in srv/CapturePointPose in the package march_shared_msgs.
typedef struct march_shared_msgs__srv__CapturePointPose_Response
{
  bool success;
  double duration;
  geometry_msgs__msg__Pose capture_point;
} march_shared_msgs__srv__CapturePointPose_Response;

// Struct for a sequence of march_shared_msgs__srv__CapturePointPose_Response.
typedef struct march_shared_msgs__srv__CapturePointPose_Response__Sequence
{
  march_shared_msgs__srv__CapturePointPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__srv__CapturePointPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__STRUCT_H_
