// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/BatteryState.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/battery_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
march_shared_msgs__msg__BatteryState__init(march_shared_msgs__msg__BatteryState * msg)
{
  if (!msg) {
    return false;
  }
  // percentage
  // voltage
  // temperature
  return true;
}

void
march_shared_msgs__msg__BatteryState__fini(march_shared_msgs__msg__BatteryState * msg)
{
  if (!msg) {
    return;
  }
  // percentage
  // voltage
  // temperature
}

march_shared_msgs__msg__BatteryState *
march_shared_msgs__msg__BatteryState__create()
{
  march_shared_msgs__msg__BatteryState * msg = (march_shared_msgs__msg__BatteryState *)malloc(sizeof(march_shared_msgs__msg__BatteryState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__BatteryState));
  bool success = march_shared_msgs__msg__BatteryState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__BatteryState__destroy(march_shared_msgs__msg__BatteryState * msg)
{
  if (msg) {
    march_shared_msgs__msg__BatteryState__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__BatteryState__Sequence__init(march_shared_msgs__msg__BatteryState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__BatteryState * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__BatteryState *)calloc(size, sizeof(march_shared_msgs__msg__BatteryState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__BatteryState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__BatteryState__fini(&data[i - 1]);
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
march_shared_msgs__msg__BatteryState__Sequence__fini(march_shared_msgs__msg__BatteryState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__BatteryState__fini(&array->data[i]);
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

march_shared_msgs__msg__BatteryState__Sequence *
march_shared_msgs__msg__BatteryState__Sequence__create(size_t size)
{
  march_shared_msgs__msg__BatteryState__Sequence * array = (march_shared_msgs__msg__BatteryState__Sequence *)malloc(sizeof(march_shared_msgs__msg__BatteryState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__BatteryState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__BatteryState__Sequence__destroy(march_shared_msgs__msg__BatteryState__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__BatteryState__Sequence__fini(array);
  }
  free(array);
}
