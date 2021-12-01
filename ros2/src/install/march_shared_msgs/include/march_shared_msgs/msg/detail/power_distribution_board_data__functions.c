// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:msg/PowerDistributionBoardData.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/msg/detail/power_distribution_board_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `hv_state`
#include "march_shared_msgs/msg/detail/high_voltage_state__functions.h"
// Member `lv_state`
#include "march_shared_msgs/msg/detail/low_voltage_state__functions.h"
// Member `battery_state`
#include "march_shared_msgs/msg/detail/battery_state__functions.h"

bool
march_shared_msgs__msg__PowerDistributionBoardData__init(march_shared_msgs__msg__PowerDistributionBoardData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    march_shared_msgs__msg__PowerDistributionBoardData__fini(msg);
    return false;
  }
  // emergency_button_state
  // pdb_current
  // hv_state
  if (!march_shared_msgs__msg__HighVoltageState__init(&msg->hv_state)) {
    march_shared_msgs__msg__PowerDistributionBoardData__fini(msg);
    return false;
  }
  // stop_button_state
  // lv_state
  if (!march_shared_msgs__msg__LowVoltageState__init(&msg->lv_state)) {
    march_shared_msgs__msg__PowerDistributionBoardData__fini(msg);
    return false;
  }
  // battery_state
  if (!march_shared_msgs__msg__BatteryState__init(&msg->battery_state)) {
    march_shared_msgs__msg__PowerDistributionBoardData__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__msg__PowerDistributionBoardData__fini(march_shared_msgs__msg__PowerDistributionBoardData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // emergency_button_state
  // pdb_current
  // hv_state
  march_shared_msgs__msg__HighVoltageState__fini(&msg->hv_state);
  // stop_button_state
  // lv_state
  march_shared_msgs__msg__LowVoltageState__fini(&msg->lv_state);
  // battery_state
  march_shared_msgs__msg__BatteryState__fini(&msg->battery_state);
}

march_shared_msgs__msg__PowerDistributionBoardData *
march_shared_msgs__msg__PowerDistributionBoardData__create()
{
  march_shared_msgs__msg__PowerDistributionBoardData * msg = (march_shared_msgs__msg__PowerDistributionBoardData *)malloc(sizeof(march_shared_msgs__msg__PowerDistributionBoardData));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__msg__PowerDistributionBoardData));
  bool success = march_shared_msgs__msg__PowerDistributionBoardData__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__msg__PowerDistributionBoardData__destroy(march_shared_msgs__msg__PowerDistributionBoardData * msg)
{
  if (msg) {
    march_shared_msgs__msg__PowerDistributionBoardData__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__msg__PowerDistributionBoardData__Sequence__init(march_shared_msgs__msg__PowerDistributionBoardData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__msg__PowerDistributionBoardData * data = NULL;
  if (size) {
    data = (march_shared_msgs__msg__PowerDistributionBoardData *)calloc(size, sizeof(march_shared_msgs__msg__PowerDistributionBoardData));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__msg__PowerDistributionBoardData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__msg__PowerDistributionBoardData__fini(&data[i - 1]);
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
march_shared_msgs__msg__PowerDistributionBoardData__Sequence__fini(march_shared_msgs__msg__PowerDistributionBoardData__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__msg__PowerDistributionBoardData__fini(&array->data[i]);
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

march_shared_msgs__msg__PowerDistributionBoardData__Sequence *
march_shared_msgs__msg__PowerDistributionBoardData__Sequence__create(size_t size)
{
  march_shared_msgs__msg__PowerDistributionBoardData__Sequence * array = (march_shared_msgs__msg__PowerDistributionBoardData__Sequence *)malloc(sizeof(march_shared_msgs__msg__PowerDistributionBoardData__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__msg__PowerDistributionBoardData__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__msg__PowerDistributionBoardData__Sequence__destroy(march_shared_msgs__msg__PowerDistributionBoardData__Sequence * array)
{
  if (array) {
    march_shared_msgs__msg__PowerDistributionBoardData__Sequence__fini(array);
  }
  free(array);
}
