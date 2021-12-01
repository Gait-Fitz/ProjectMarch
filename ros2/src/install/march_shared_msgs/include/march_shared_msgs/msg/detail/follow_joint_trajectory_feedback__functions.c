// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryFeedback.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `desired`
// Member `actual`
// Member `error`
#include "trajectory_msgs/msg/detail/joint_trajectory_point__functions.h"

bool
march_shared_msgs__msg__FollowJointTrajectoryFeedback__init(march_shared_msgs__msg__FollowJointTrajectoryFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(msg);
    return false;
  }
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->joint_names, 0)) {
    march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(msg);
    return false;
  }
  // desired
  if (!trajectory_msgs__msg__JointTrajectoryPoint__init(&msg->desired)) {
    march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(msg);
    return false;
  }
  // actual
  if (!trajectory_msgs__msg__JointTrajectoryPoint__init(&msg->actual)) {
    march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(msg);
    return false;
  }
  // error
  if (!trajectory_msgs__msg__JointTrajectoryPoint__init(&msg->error)) {
    march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(march_shared_msgs__msg__FollowJointTrajectoryFeedback * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // joint_names
  rosidl_runtime_c__String__Sequence__fini(&msg->joint_names);
  // desired
  trajectory_msgs__msg__JointTrajectoryPoint__fini(&msg->desired);
  // actual
  trajectory_msgs__msg__JointTrajectoryPoint__fini(&msg->actual);
  // error
  trajectory_msgs__msg__JointTrajectoryPoint__fini(&msg->error);
}

march_shared_msgs__msg__FollowJointTrajectoryFeedback *
march_shared_msgs__msg__FollowJointTrajectoryFeedback__create()
{
  march_shared_msgs__msg__FollowJointTrajectoryFeedback * msg = (march_shared_msgs__msg__FollowJointTrajectoryFeedback *)malloc(sizeof(march_shared_msgs__msg__FollowJointTrajectoryFeedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__FollowJointTrajectoryFeedback));
  bool success = march_shared_msgs__msg__FollowJointTrajectoryFeedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__FollowJointTrajectoryFeedback__destroy(march_shared_msgs__msg__FollowJointTrajectoryFeedback * msg)
{
  if (msg) {
    march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence__init(march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__FollowJointTrajectoryFeedback * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__FollowJointTrajectoryFeedback *)calloc(size, sizeof(march_shared_msgs__msg__FollowJointTrajectoryFeedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__FollowJointTrajectoryFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence__fini(march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence *
march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence__create(size_t size)
{
  march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence * array = (march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence *)malloc(sizeof(march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence__destroy(march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__FollowJointTrajectoryFeedback__Sequence__fini(array);
  }
  free(array);
}
