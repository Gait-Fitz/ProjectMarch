// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/GaitInstructionResponse.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/gait_instruction_response__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
march_shared_msgs__msg__GaitInstructionResponse__init(march_shared_msgs__msg__GaitInstructionResponse * msg)
{
  if (!msg) {
    return false;
  }
  // result
  return true;
}

void
march_shared_msgs__msg__GaitInstructionResponse__fini(march_shared_msgs__msg__GaitInstructionResponse * msg)
{
  if (!msg) {
    return;
  }
  // result
}

march_shared_msgs__msg__GaitInstructionResponse *
march_shared_msgs__msg__GaitInstructionResponse__create()
{
  march_shared_msgs__msg__GaitInstructionResponse * msg = (march_shared_msgs__msg__GaitInstructionResponse *)malloc(sizeof(march_shared_msgs__msg__GaitInstructionResponse));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__GaitInstructionResponse));
  bool success = march_shared_msgs__msg__GaitInstructionResponse__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__GaitInstructionResponse__destroy(march_shared_msgs__msg__GaitInstructionResponse * msg)
{
  if (msg) {
    march_shared_msgs__msg__GaitInstructionResponse__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__GaitInstructionResponse__Sequence__init(march_shared_msgs__msg__GaitInstructionResponse__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__GaitInstructionResponse * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__GaitInstructionResponse *)calloc(size, sizeof(march_shared_msgs__msg__GaitInstructionResponse));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__GaitInstructionResponse__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__GaitInstructionResponse__fini(&data[i - 1]);
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
march_shared_msgs__msg__GaitInstructionResponse__Sequence__fini(march_shared_msgs__msg__GaitInstructionResponse__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__GaitInstructionResponse__fini(&array->data[i]);
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

march_shared_msgs__msg__GaitInstructionResponse__Sequence *
march_shared_msgs__msg__GaitInstructionResponse__Sequence__create(size_t size)
{
  march_shared_msgs__msg__GaitInstructionResponse__Sequence * array = (march_shared_msgs__msg__GaitInstructionResponse__Sequence *)malloc(sizeof(march_shared_msgs__msg__GaitInstructionResponse__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__GaitInstructionResponse__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__GaitInstructionResponse__Sequence__destroy(march_shared_msgs__msg__GaitInstructionResponse__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__GaitInstructionResponse__Sequence__fini(array);
  }
  free(array);
}
