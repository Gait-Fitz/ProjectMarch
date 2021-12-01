// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionFeedback.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `status`
#include "actionlib_msgs/msg/detail/goal_status__functions.h"
// Member `feedback`
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_feedback__functions.h"

bool
march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__init(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__fini(msg);
    return false;
  }
  // status
  if (!actionlib_msgs__msg__GoalStatus__init(&msg->status)) {
    march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__fini(msg);
    return false;
  }
  // feedback
  if (!march_shared_msgs__msg__FollowJointTrajectoryFeedback__init(&msg->feedback)) {
    march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__fini(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // status
  actionlib_msgs__msg__GoalStatus__fini(&msg->status);
  // feedback
  march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(&msg->feedback);
}

march_shared_msgs__msg__FollowJointTrajectoryActionFeedback *
march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__create()
{
  march_shared_msgs__msg__FollowJointTrajectoryActionFeedback * msg = (march_shared_msgs__msg__FollowJointTrajectoryActionFeedback *)malloc(sizeof(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback));
  bool success = march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__destroy(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback * msg)
{
  if (msg) {
    march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence__init(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__FollowJointTrajectoryActionFeedback * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__FollowJointTrajectoryActionFeedback *)calloc(size, sizeof(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__fini(&data[i - 1]);
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
march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence__fini(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__fini(&array->data[i]);
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

march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence *
march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence__create(size_t size)
{
  march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence * array = (march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence *)malloc(sizeof(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence__destroy(march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__FollowJointTrajectoryActionFeedback__Sequence__fini(array);
  }
  free(array);
}
