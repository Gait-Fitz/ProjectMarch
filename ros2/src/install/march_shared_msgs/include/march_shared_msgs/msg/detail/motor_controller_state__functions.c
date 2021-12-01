// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/MotorControllerState.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/motor_controller_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `joint_names`
// Member `error_status`
// Member `operational_state`
#include "rosidl_runtime_c/string_functions.h"
// Member `motor_current`
// Member `motor_voltage`
// Member `temperature`
// Member `absolute_position_iu`
// Member `incremental_position_iu`
// Member `absolute_velocity_iu`
// Member `incremental_velocity_iu`
// Member `absolute_position`
// Member `incremental_position`
// Member `absolute_velocity`
// Member `incremental_velocity`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
march_shared_msgs__msg__MotorControllerState__init(march_shared_msgs__msg__MotorControllerState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->joint_names, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // error_status
  if (!rosidl_runtime_c__String__Sequence__init(&msg->error_status, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // operational_state
  if (!rosidl_runtime_c__String__Sequence__init(&msg->operational_state, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // motor_current
  if (!rosidl_runtime_c__float__Sequence__init(&msg->motor_current, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // motor_voltage
  if (!rosidl_runtime_c__float__Sequence__init(&msg->motor_voltage, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // temperature
  if (!rosidl_runtime_c__float__Sequence__init(&msg->temperature, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // absolute_position_iu
  if (!rosidl_runtime_c__float__Sequence__init(&msg->absolute_position_iu, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // incremental_position_iu
  if (!rosidl_runtime_c__float__Sequence__init(&msg->incremental_position_iu, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // absolute_velocity_iu
  if (!rosidl_runtime_c__float__Sequence__init(&msg->absolute_velocity_iu, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // incremental_velocity_iu
  if (!rosidl_runtime_c__float__Sequence__init(&msg->incremental_velocity_iu, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // absolute_position
  if (!rosidl_runtime_c__float__Sequence__init(&msg->absolute_position, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // incremental_position
  if (!rosidl_runtime_c__float__Sequence__init(&msg->incremental_position, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // absolute_velocity
  if (!rosidl_runtime_c__float__Sequence__init(&msg->absolute_velocity, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  // incremental_velocity
  if (!rosidl_runtime_c__float__Sequence__init(&msg->incremental_velocity, 0)) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__msg__MotorControllerState__fini(march_shared_msgs__msg__MotorControllerState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // joint_names
  rosidl_runtime_c__String__Sequence__fini(&msg->joint_names);
  // error_status
  rosidl_runtime_c__String__Sequence__fini(&msg->error_status);
  // operational_state
  rosidl_runtime_c__String__Sequence__fini(&msg->operational_state);
  // motor_current
  rosidl_runtime_c__float__Sequence__fini(&msg->motor_current);
  // motor_voltage
  rosidl_runtime_c__float__Sequence__fini(&msg->motor_voltage);
  // temperature
  rosidl_runtime_c__float__Sequence__fini(&msg->temperature);
  // absolute_position_iu
  rosidl_runtime_c__float__Sequence__fini(&msg->absolute_position_iu);
  // incremental_position_iu
  rosidl_runtime_c__float__Sequence__fini(&msg->incremental_position_iu);
  // absolute_velocity_iu
  rosidl_runtime_c__float__Sequence__fini(&msg->absolute_velocity_iu);
  // incremental_velocity_iu
  rosidl_runtime_c__float__Sequence__fini(&msg->incremental_velocity_iu);
  // absolute_position
  rosidl_runtime_c__float__Sequence__fini(&msg->absolute_position);
  // incremental_position
  rosidl_runtime_c__float__Sequence__fini(&msg->incremental_position);
  // absolute_velocity
  rosidl_runtime_c__float__Sequence__fini(&msg->absolute_velocity);
  // incremental_velocity
  rosidl_runtime_c__float__Sequence__fini(&msg->incremental_velocity);
}

march_shared_msgs__msg__MotorControllerState *
march_shared_msgs__msg__MotorControllerState__create()
{
  march_shared_msgs__msg__MotorControllerState * msg = (march_shared_msgs__msg__MotorControllerState *)malloc(sizeof(march_shared_msgs__msg__MotorControllerState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__MotorControllerState));
  bool success = march_shared_msgs__msg__MotorControllerState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__MotorControllerState__destroy(march_shared_msgs__msg__MotorControllerState * msg)
{
  if (msg) {
    march_shared_msgs__msg__MotorControllerState__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__MotorControllerState__Sequence__init(march_shared_msgs__msg__MotorControllerState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__MotorControllerState * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__MotorControllerState *)calloc(size, sizeof(march_shared_msgs__msg__MotorControllerState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__MotorControllerState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__MotorControllerState__fini(&data[i - 1]);
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
march_shared_msgs__msg__MotorControllerState__Sequence__fini(march_shared_msgs__msg__MotorControllerState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__MotorControllerState__fini(&array->data[i]);
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

march_shared_msgs__msg__MotorControllerState__Sequence *
march_shared_msgs__msg__MotorControllerState__Sequence__create(size_t size)
{
  march_shared_msgs__msg__MotorControllerState__Sequence * array = (march_shared_msgs__msg__MotorControllerState__Sequence *)malloc(sizeof(march_shared_msgs__msg__MotorControllerState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__MotorControllerState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__MotorControllerState__Sequence__destroy(march_shared_msgs__msg__MotorControllerState__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__MotorControllerState__Sequence__fini(array);
  }
  free(array);
}
