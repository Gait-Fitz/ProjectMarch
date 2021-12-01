// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/MpcArray.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/mpc_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `array`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
march_shared_msgs__msg__MpcArray__init(march_shared_msgs__msg__MpcArray * msg)
{
  if (!msg) {
    return false;
  }
  // array
  if (!rosidl_runtime_c__double__Sequence__init(&msg->array, 0)) {
    march_shared_msgs__msg__MpcArray__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__msg__MpcArray__fini(march_shared_msgs__msg__MpcArray * msg)
{
  if (!msg) {
    return;
  }
  // array
  rosidl_runtime_c__double__Sequence__fini(&msg->array);
}

march_shared_msgs__msg__MpcArray *
march_shared_msgs__msg__MpcArray__create()
{
  march_shared_msgs__msg__MpcArray * msg = (march_shared_msgs__msg__MpcArray *)malloc(sizeof(march_shared_msgs__msg__MpcArray));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__MpcArray));
  bool success = march_shared_msgs__msg__MpcArray__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__MpcArray__destroy(march_shared_msgs__msg__MpcArray * msg)
{
  if (msg) {
    march_shared_msgs__msg__MpcArray__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__MpcArray__Sequence__init(march_shared_msgs__msg__MpcArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__MpcArray * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__MpcArray *)calloc(size, sizeof(march_shared_msgs__msg__MpcArray));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__MpcArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__MpcArray__fini(&data[i - 1]);
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
march_shared_msgs__msg__MpcArray__Sequence__fini(march_shared_msgs__msg__MpcArray__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__MpcArray__fini(&array->data[i]);
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

march_shared_msgs__msg__MpcArray__Sequence *
march_shared_msgs__msg__MpcArray__Sequence__create(size_t size)
{
  march_shared_msgs__msg__MpcArray__Sequence * array = (march_shared_msgs__msg__MpcArray__Sequence *)malloc(sizeof(march_shared_msgs__msg__MpcArray__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__MpcArray__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__MpcArray__Sequence__destroy(march_shared_msgs__msg__MpcArray__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__MpcArray__Sequence__fini(array);
  }
  free(array);
}
