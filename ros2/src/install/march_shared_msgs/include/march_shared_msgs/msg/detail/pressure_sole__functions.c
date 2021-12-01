// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/PressureSole.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/pressure_sole__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pressure_soles_time`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `pressure_left`
// Member `pressure_right`
// Member `cop_left`
// Member `cop_right`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
march_shared_msgs__msg__PressureSole__init(march_shared_msgs__msg__PressureSole * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    march_shared_msgs__msg__PressureSole__fini(msg);
    return false;
  }
  // pressure_soles_time
  if (!builtin_interfaces__msg__Time__init(&msg->pressure_soles_time)) {
    march_shared_msgs__msg__PressureSole__fini(msg);
    return false;
  }
  // total_force_left
  // total_force_right
  // pressure_left
  if (!rosidl_runtime_c__float__Sequence__init(&msg->pressure_left, 0)) {
    march_shared_msgs__msg__PressureSole__fini(msg);
    return false;
  }
  // pressure_right
  if (!rosidl_runtime_c__float__Sequence__init(&msg->pressure_right, 0)) {
    march_shared_msgs__msg__PressureSole__fini(msg);
    return false;
  }
  // cop_left
  if (!rosidl_runtime_c__float__Sequence__init(&msg->cop_left, 0)) {
    march_shared_msgs__msg__PressureSole__fini(msg);
    return false;
  }
  // cop_right
  if (!rosidl_runtime_c__float__Sequence__init(&msg->cop_right, 0)) {
    march_shared_msgs__msg__PressureSole__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__msg__PressureSole__fini(march_shared_msgs__msg__PressureSole * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pressure_soles_time
  builtin_interfaces__msg__Time__fini(&msg->pressure_soles_time);
  // total_force_left
  // total_force_right
  // pressure_left
  rosidl_runtime_c__float__Sequence__fini(&msg->pressure_left);
  // pressure_right
  rosidl_runtime_c__float__Sequence__fini(&msg->pressure_right);
  // cop_left
  rosidl_runtime_c__float__Sequence__fini(&msg->cop_left);
  // cop_right
  rosidl_runtime_c__float__Sequence__fini(&msg->cop_right);
}

march_shared_msgs__msg__PressureSole *
march_shared_msgs__msg__PressureSole__create()
{
  march_shared_msgs__msg__PressureSole * msg = (march_shared_msgs__msg__PressureSole *)malloc(sizeof(march_shared_msgs__msg__PressureSole));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__PressureSole));
  bool success = march_shared_msgs__msg__PressureSole__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__PressureSole__destroy(march_shared_msgs__msg__PressureSole * msg)
{
  if (msg) {
    march_shared_msgs__msg__PressureSole__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__PressureSole__Sequence__init(march_shared_msgs__msg__PressureSole__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__PressureSole * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__PressureSole *)calloc(size, sizeof(march_shared_msgs__msg__PressureSole));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__PressureSole__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__PressureSole__fini(&data[i - 1]);
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
march_shared_msgs__msg__PressureSole__Sequence__fini(march_shared_msgs__msg__PressureSole__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__PressureSole__fini(&array->data[i]);
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

march_shared_msgs__msg__PressureSole__Sequence *
march_shared_msgs__msg__PressureSole__Sequence__create(size_t size)
{
  march_shared_msgs__msg__PressureSole__Sequence * array = (march_shared_msgs__msg__PressureSole__Sequence *)malloc(sizeof(march_shared_msgs__msg__PressureSole__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__PressureSole__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__PressureSole__Sequence__destroy(march_shared_msgs__msg__PressureSole__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__PressureSole__Sequence__fini(array);
  }
  free(array);
}
