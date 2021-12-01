// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/CurrentGait.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/current_gait__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `gait`
// Member `subgait`
// Member `version`
// Member `gait_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `duration`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
march_shared_msgs__msg__CurrentGait__init(march_shared_msgs__msg__CurrentGait * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    march_shared_msgs__msg__CurrentGait__fini(msg);
    return false;
  }
  // gait
  if (!rosidl_runtime_c__String__init(&msg->gait)) {
    march_shared_msgs__msg__CurrentGait__fini(msg);
    return false;
  }
  // subgait
  if (!rosidl_runtime_c__String__init(&msg->subgait)) {
    march_shared_msgs__msg__CurrentGait__fini(msg);
    return false;
  }
  // version
  if (!rosidl_runtime_c__String__init(&msg->version)) {
    march_shared_msgs__msg__CurrentGait__fini(msg);
    return false;
  }
  // duration
  if (!builtin_interfaces__msg__Duration__init(&msg->duration)) {
    march_shared_msgs__msg__CurrentGait__fini(msg);
    return false;
  }
  // gait_type
  if (!rosidl_runtime_c__String__init(&msg->gait_type)) {
    march_shared_msgs__msg__CurrentGait__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__msg__CurrentGait__fini(march_shared_msgs__msg__CurrentGait * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // gait
  rosidl_runtime_c__String__fini(&msg->gait);
  // subgait
  rosidl_runtime_c__String__fini(&msg->subgait);
  // version
  rosidl_runtime_c__String__fini(&msg->version);
  // duration
  builtin_interfaces__msg__Duration__fini(&msg->duration);
  // gait_type
  rosidl_runtime_c__String__fini(&msg->gait_type);
}

march_shared_msgs__msg__CurrentGait *
march_shared_msgs__msg__CurrentGait__create()
{
  march_shared_msgs__msg__CurrentGait * msg = (march_shared_msgs__msg__CurrentGait *)malloc(sizeof(march_shared_msgs__msg__CurrentGait));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__CurrentGait));
  bool success = march_shared_msgs__msg__CurrentGait__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__CurrentGait__destroy(march_shared_msgs__msg__CurrentGait * msg)
{
  if (msg) {
    march_shared_msgs__msg__CurrentGait__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__CurrentGait__Sequence__init(march_shared_msgs__msg__CurrentGait__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__CurrentGait * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__CurrentGait *)calloc(size, sizeof(march_shared_msgs__msg__CurrentGait));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__CurrentGait__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__CurrentGait__fini(&data[i - 1]);
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
march_shared_msgs__msg__CurrentGait__Sequence__fini(march_shared_msgs__msg__CurrentGait__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__CurrentGait__fini(&array->data[i]);
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

march_shared_msgs__msg__CurrentGait__Sequence *
march_shared_msgs__msg__CurrentGait__Sequence__create(size_t size)
{
  march_shared_msgs__msg__CurrentGait__Sequence * array = (march_shared_msgs__msg__CurrentGait__Sequence *)malloc(sizeof(march_shared_msgs__msg__CurrentGait__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__CurrentGait__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__CurrentGait__Sequence__destroy(march_shared_msgs__msg__CurrentGait__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__CurrentGait__Sequence__fini(array);
  }
  free(array);
}
