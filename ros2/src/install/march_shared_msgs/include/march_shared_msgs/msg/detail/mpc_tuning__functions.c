// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/MpcTuning.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/mpc_tuning__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `q_weights`
// Member `r_weights`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
march_shared_msgs__msg__MpcTuning__init(march_shared_msgs__msg__MpcTuning * msg)
{
  if (!msg) {
    return false;
  }
  // q_weights
  if (!rosidl_runtime_c__float__Sequence__init(&msg->q_weights, 0)) {
    march_shared_msgs__msg__MpcTuning__fini(msg);
    return false;
  }
  // r_weights
  if (!rosidl_runtime_c__float__Sequence__init(&msg->r_weights, 0)) {
    march_shared_msgs__msg__MpcTuning__fini(msg);
    return false;
  }
  // horizon
  return true;
}

void
march_shared_msgs__msg__MpcTuning__fini(march_shared_msgs__msg__MpcTuning * msg)
{
  if (!msg) {
    return;
  }
  // q_weights
  rosidl_runtime_c__float__Sequence__fini(&msg->q_weights);
  // r_weights
  rosidl_runtime_c__float__Sequence__fini(&msg->r_weights);
  // horizon
}

march_shared_msgs__msg__MpcTuning *
march_shared_msgs__msg__MpcTuning__create()
{
  march_shared_msgs__msg__MpcTuning * msg = (march_shared_msgs__msg__MpcTuning *)malloc(sizeof(march_shared_msgs__msg__MpcTuning));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__MpcTuning));
  bool success = march_shared_msgs__msg__MpcTuning__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__MpcTuning__destroy(march_shared_msgs__msg__MpcTuning * msg)
{
  if (msg) {
    march_shared_msgs__msg__MpcTuning__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__MpcTuning__Sequence__init(march_shared_msgs__msg__MpcTuning__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__MpcTuning * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__MpcTuning *)calloc(size, sizeof(march_shared_msgs__msg__MpcTuning));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__MpcTuning__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__MpcTuning__fini(&data[i - 1]);
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
march_shared_msgs__msg__MpcTuning__Sequence__fini(march_shared_msgs__msg__MpcTuning__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__MpcTuning__fini(&array->data[i]);
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

march_shared_msgs__msg__MpcTuning__Sequence *
march_shared_msgs__msg__MpcTuning__Sequence__create(size_t size)
{
  march_shared_msgs__msg__MpcTuning__Sequence * array = (march_shared_msgs__msg__MpcTuning__Sequence *)malloc(sizeof(march_shared_msgs__msg__MpcTuning__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__MpcTuning__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__MpcTuning__Sequence__destroy(march_shared_msgs__msg__MpcTuning__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__MpcTuning__Sequence__fini(array);
  }
  free(array);
}
