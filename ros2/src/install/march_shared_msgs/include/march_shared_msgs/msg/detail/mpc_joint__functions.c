// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/MpcJoint.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/mpc_joint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `estimation`
// Member `reference`
#include "march_shared_msgs/msg/detail/mpc_state_vectors__functions.h"
// Member `tuning`
#include "march_shared_msgs/msg/detail/mpc_tuning__functions.h"

bool
march_shared_msgs__msg__MpcJoint__init(march_shared_msgs__msg__MpcJoint * msg)
{
  if (!msg) {
    return false;
  }
  // estimation
  if (!march_shared_msgs__msg__MpcStateVectors__init(&msg->estimation)) {
    march_shared_msgs__msg__MpcJoint__fini(msg);
    return false;
  }
  // reference
  if (!march_shared_msgs__msg__MpcStateVectors__init(&msg->reference)) {
    march_shared_msgs__msg__MpcJoint__fini(msg);
    return false;
  }
  // tuning
  if (!march_shared_msgs__msg__MpcTuning__init(&msg->tuning)) {
    march_shared_msgs__msg__MpcJoint__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__msg__MpcJoint__fini(march_shared_msgs__msg__MpcJoint * msg)
{
  if (!msg) {
    return;
  }
  // estimation
  march_shared_msgs__msg__MpcStateVectors__fini(&msg->estimation);
  // reference
  march_shared_msgs__msg__MpcStateVectors__fini(&msg->reference);
  // tuning
  march_shared_msgs__msg__MpcTuning__fini(&msg->tuning);
}

march_shared_msgs__msg__MpcJoint *
march_shared_msgs__msg__MpcJoint__create()
{
  march_shared_msgs__msg__MpcJoint * msg = (march_shared_msgs__msg__MpcJoint *)malloc(sizeof(march_shared_msgs__msg__MpcJoint));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__MpcJoint));
  bool success = march_shared_msgs__msg__MpcJoint__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__MpcJoint__destroy(march_shared_msgs__msg__MpcJoint * msg)
{
  if (msg) {
    march_shared_msgs__msg__MpcJoint__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__MpcJoint__Sequence__init(march_shared_msgs__msg__MpcJoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__MpcJoint * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__MpcJoint *)calloc(size, sizeof(march_shared_msgs__msg__MpcJoint));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__MpcJoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__MpcJoint__fini(&data[i - 1]);
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
march_shared_msgs__msg__MpcJoint__Sequence__fini(march_shared_msgs__msg__MpcJoint__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__MpcJoint__fini(&array->data[i]);
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

march_shared_msgs__msg__MpcJoint__Sequence *
march_shared_msgs__msg__MpcJoint__Sequence__create(size_t size)
{
  march_shared_msgs__msg__MpcJoint__Sequence * array = (march_shared_msgs__msg__MpcJoint__Sequence *)malloc(sizeof(march_shared_msgs__msg__MpcJoint__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__MpcJoint__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__MpcJoint__Sequence__destroy(march_shared_msgs__msg__MpcJoint__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__MpcJoint__Sequence__fini(array);
  }
  free(array);
}
