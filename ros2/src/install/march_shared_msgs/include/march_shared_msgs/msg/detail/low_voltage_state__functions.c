// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/LowVoltageState.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/low_voltage_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
march_shared_msgs__msg__LowVoltageState__init(march_shared_msgs__msg__LowVoltageState * msg)
{
  if (!msg) {
    return false;
  }
  // lv1_current
  // lv2_current
  // lv1_ok
  // lv2_ok
  return true;
}

void
march_shared_msgs__msg__LowVoltageState__fini(march_shared_msgs__msg__LowVoltageState * msg)
{
  if (!msg) {
    return;
  }
  // lv1_current
  // lv2_current
  // lv1_ok
  // lv2_ok
}

march_shared_msgs__msg__LowVoltageState *
march_shared_msgs__msg__LowVoltageState__create()
{
  march_shared_msgs__msg__LowVoltageState * msg = (march_shared_msgs__msg__LowVoltageState *)malloc(sizeof(march_shared_msgs__msg__LowVoltageState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__LowVoltageState));
  bool success = march_shared_msgs__msg__LowVoltageState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__LowVoltageState__destroy(march_shared_msgs__msg__LowVoltageState * msg)
{
  if (msg) {
    march_shared_msgs__msg__LowVoltageState__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__LowVoltageState__Sequence__init(march_shared_msgs__msg__LowVoltageState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__LowVoltageState * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__LowVoltageState *)calloc(size, sizeof(march_shared_msgs__msg__LowVoltageState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__LowVoltageState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__LowVoltageState__fini(&data[i - 1]);
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
march_shared_msgs__msg__LowVoltageState__Sequence__fini(march_shared_msgs__msg__LowVoltageState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__LowVoltageState__fini(&array->data[i]);
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

march_shared_msgs__msg__LowVoltageState__Sequence *
march_shared_msgs__msg__LowVoltageState__Sequence__create(size_t size)
{
  march_shared_msgs__msg__LowVoltageState__Sequence * array = (march_shared_msgs__msg__LowVoltageState__Sequence *)malloc(sizeof(march_shared_msgs__msg__LowVoltageState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__LowVoltageState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__LowVoltageState__Sequence__destroy(march_shared_msgs__msg__LowVoltageState__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__LowVoltageState__Sequence__fini(array);
  }
  free(array);
}
