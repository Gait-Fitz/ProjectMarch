// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/CurrentState.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/current_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state`
#include "rosidl_runtime_c/string_functions.h"

bool
march_shared_msgs__msg__CurrentState__init(march_shared_msgs__msg__CurrentState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    march_shared_msgs__msg__CurrentState__fini(msg);
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    march_shared_msgs__msg__CurrentState__fini(msg);
    return false;
  }
  // state_type
  return true;
}

void
march_shared_msgs__msg__CurrentState__fini(march_shared_msgs__msg__CurrentState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // state_type
}

march_shared_msgs__msg__CurrentState *
march_shared_msgs__msg__CurrentState__create()
{
  march_shared_msgs__msg__CurrentState * msg = (march_shared_msgs__msg__CurrentState *)malloc(sizeof(march_shared_msgs__msg__CurrentState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__CurrentState));
  bool success = march_shared_msgs__msg__CurrentState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__CurrentState__destroy(march_shared_msgs__msg__CurrentState * msg)
{
  if (msg) {
    march_shared_msgs__msg__CurrentState__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__CurrentState__Sequence__init(march_shared_msgs__msg__CurrentState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__CurrentState * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__CurrentState *)calloc(size, sizeof(march_shared_msgs__msg__CurrentState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__CurrentState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__CurrentState__fini(&data[i - 1]);
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
march_shared_msgs__msg__CurrentState__Sequence__fini(march_shared_msgs__msg__CurrentState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__CurrentState__fini(&array->data[i]);
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

march_shared_msgs__msg__CurrentState__Sequence *
march_shared_msgs__msg__CurrentState__Sequence__create(size_t size)
{
  march_shared_msgs__msg__CurrentState__Sequence * array = (march_shared_msgs__msg__CurrentState__Sequence *)malloc(sizeof(march_shared_msgs__msg__CurrentState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__CurrentState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__CurrentState__Sequence__destroy(march_shared_msgs__msg__CurrentState__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__CurrentState__Sequence__fini(array);
  }
  free(array);
}
