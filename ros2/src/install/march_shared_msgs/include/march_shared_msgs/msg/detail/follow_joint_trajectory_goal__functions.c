// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryGoal.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_goal__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `trajectory`
#include "trajectory_msgs/msg/detail/joint_trajectory__functions.h"
// Member `path_tolerance`
// Member `goal_tolerance`
#include "control_msgs/msg/detail/joint_tolerance__functions.h"
// Member `goal_time_tolerance`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
march_shared_msgs__msg__FollowJointTrajectoryGoal__init(march_shared_msgs__msg__FollowJointTrajectoryGoal * msg)
{
  if (!msg) {
    return false;
  }
  // trajectory
  if (!trajectory_msgs__msg__JointTrajectory__init(&msg->trajectory)) {
    march_shared_msgs__msg__FollowJointTrajectoryGoal__fini(msg);
    return false;
  }
  // path_tolerance
  if (!control_msgs__msg__JointTolerance__Sequence__init(&msg->path_tolerance, 0)) {
    march_shared_msgs__msg__FollowJointTrajectoryGoal__fini(msg);
    return false;
  }
  // goal_tolerance
  if (!control_msgs__msg__JointTolerance__Sequence__init(&msg->goal_tolerance, 0)) {
    march_shared_msgs__msg__FollowJointTrajectoryGoal__fini(msg);
    return false;
  }
  // goal_time_tolerance
  if (!builtin_interfaces__msg__Duration__init(&msg->goal_time_tolerance)) {
    march_shared_msgs__msg__FollowJointTrajectoryGoal__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__msg__FollowJointTrajectoryGoal__fini(march_shared_msgs__msg__FollowJointTrajectoryGoal * msg)
{
  if (!msg) {
    return;
  }
  // trajectory
  trajectory_msgs__msg__JointTrajectory__fini(&msg->trajectory);
  // path_tolerance
  control_msgs__msg__JointTolerance__Sequence__fini(&msg->path_tolerance);
  // goal_tolerance
  control_msgs__msg__JointTolerance__Sequence__fini(&msg->goal_tolerance);
  // goal_time_tolerance
  builtin_interfaces__msg__Duration__fini(&msg->goal_time_tolerance);
}

march_shared_msgs__msg__FollowJointTrajectoryGoal *
march_shared_msgs__msg__FollowJointTrajectoryGoal__create()
{
  march_shared_msgs__msg__FollowJointTrajectoryGoal * msg = (march_shared_msgs__msg__FollowJointTrajectoryGoal *)malloc(sizeof(march_shared_msgs__msg__FollowJointTrajectoryGoal));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__FollowJointTrajectoryGoal));
  bool success = march_shared_msgs__msg__FollowJointTrajectoryGoal__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__FollowJointTrajectoryGoal__destroy(march_shared_msgs__msg__FollowJointTrajectoryGoal * msg)
{
  if (msg) {
    march_shared_msgs__msg__FollowJointTrajectoryGoal__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence__init(march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__FollowJointTrajectoryGoal * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__FollowJointTrajectoryGoal *)calloc(size, sizeof(march_shared_msgs__msg__FollowJointTrajectoryGoal));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__FollowJointTrajectoryGoal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__FollowJointTrajectoryGoal__fini(&data[i - 1]);
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
march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence__fini(march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__FollowJointTrajectoryGoal__fini(&array->data[i]);
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

march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence *
march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence__create(size_t size)
{
  march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence * array = (march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence *)malloc(sizeof(march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence__destroy(march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__FollowJointTrajectoryGoal__Sequence__fini(array);
  }
  free(array);
}
