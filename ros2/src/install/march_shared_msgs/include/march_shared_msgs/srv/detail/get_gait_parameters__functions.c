// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from march_shared_msgs:srv/GetGaitParameters.idl
// generated code does not contain a copyright notice
#include "march_shared_msgs/srv/detail/get_gait_parameters__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Include directives for member types
// Member `subgait_name`
#include "rosidl_runtime_c/string_functions.h"

bool
march_shared_msgs__srv__GetGaitParameters_Request__init(march_shared_msgs__srv__GetGaitParameters_Request * msg)
{
  if (!msg) {
    return false;
  }
  // realsense_category
  // camera_to_use
  // subgait_name
  if (!rosidl_runtime_c__String__init(&msg->subgait_name)) {
    march_shared_msgs__srv__GetGaitParameters_Request__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__srv__GetGaitParameters_Request__fini(march_shared_msgs__srv__GetGaitParameters_Request * msg)
{
  if (!msg) {
    return;
  }
  // realsense_category
  // camera_to_use
  // subgait_name
  rosidl_runtime_c__String__fini(&msg->subgait_name);
}

march_shared_msgs__srv__GetGaitParameters_Request *
march_shared_msgs__srv__GetGaitParameters_Request__create()
{
  march_shared_msgs__srv__GetGaitParameters_Request * msg = (march_shared_msgs__srv__GetGaitParameters_Request *)malloc(sizeof(march_shared_msgs__srv__GetGaitParameters_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__srv__GetGaitParameters_Request));
  bool success = march_shared_msgs__srv__GetGaitParameters_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__srv__GetGaitParameters_Request__destroy(march_shared_msgs__srv__GetGaitParameters_Request * msg)
{
  if (msg) {
    march_shared_msgs__srv__GetGaitParameters_Request__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__srv__GetGaitParameters_Request__Sequence__init(march_shared_msgs__srv__GetGaitParameters_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__srv__GetGaitParameters_Request * data = NULL;
  if (size) {
    data = (march_shared_msgs__srv__GetGaitParameters_Request *)calloc(size, sizeof(march_shared_msgs__srv__GetGaitParameters_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__srv__GetGaitParameters_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__srv__GetGaitParameters_Request__fini(&data[i - 1]);
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
march_shared_msgs__srv__GetGaitParameters_Request__Sequence__fini(march_shared_msgs__srv__GetGaitParameters_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__srv__GetGaitParameters_Request__fini(&array->data[i]);
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

march_shared_msgs__srv__GetGaitParameters_Request__Sequence *
march_shared_msgs__srv__GetGaitParameters_Request__Sequence__create(size_t size)
{
  march_shared_msgs__srv__GetGaitParameters_Request__Sequence * array = (march_shared_msgs__srv__GetGaitParameters_Request__Sequence *)malloc(sizeof(march_shared_msgs__srv__GetGaitParameters_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__srv__GetGaitParameters_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__srv__GetGaitParameters_Request__Sequence__destroy(march_shared_msgs__srv__GetGaitParameters_Request__Sequence * array)
{
  if (array) {
    march_shared_msgs__srv__GetGaitParameters_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `error_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `gait_parameters`
#include "march_shared_msgs/msg/detail/gait_parameters__functions.h"

bool
march_shared_msgs__srv__GetGaitParameters_Response__init(march_shared_msgs__srv__GetGaitParameters_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    march_shared_msgs__srv__GetGaitParameters_Response__fini(msg);
    return false;
  }
  // gait_parameters
  if (!march_shared_msgs__msg__GaitParameters__init(&msg->gait_parameters)) {
    march_shared_msgs__srv__GetGaitParameters_Response__fini(msg);
    return false;
  }
  return true;
}

void
march_shared_msgs__srv__GetGaitParameters_Response__fini(march_shared_msgs__srv__GetGaitParameters_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // gait_parameters
  march_shared_msgs__msg__GaitParameters__fini(&msg->gait_parameters);
}

march_shared_msgs__srv__GetGaitParameters_Response *
march_shared_msgs__srv__GetGaitParameters_Response__create()
{
  march_shared_msgs__srv__GetGaitParameters_Response * msg = (march_shared_msgs__srv__GetGaitParameters_Response *)malloc(sizeof(march_shared_msgs__srv__GetGaitParameters_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(march_shared_msgs__srv__GetGaitParameters_Response));
  bool success = march_shared_msgs__srv__GetGaitParameters_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
march_shared_msgs__srv__GetGaitParameters_Response__destroy(march_shared_msgs__srv__GetGaitParameters_Response * msg)
{
  if (msg) {
    march_shared_msgs__srv__GetGaitParameters_Response__fini(msg);
  }
  free(msg);
}


bool
march_shared_msgs__srv__GetGaitParameters_Response__Sequence__init(march_shared_msgs__srv__GetGaitParameters_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  march_shared_msgs__srv__GetGaitParameters_Response * data = NULL;
  if (size) {
    data = (march_shared_msgs__srv__GetGaitParameters_Response *)calloc(size, sizeof(march_shared_msgs__srv__GetGaitParameters_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = march_shared_msgs__srv__GetGaitParameters_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        march_shared_msgs__srv__GetGaitParameters_Response__fini(&data[i - 1]);
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
march_shared_msgs__srv__GetGaitParameters_Response__Sequence__fini(march_shared_msgs__srv__GetGaitParameters_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      march_shared_msgs__srv__GetGaitParameters_Response__fini(&array->data[i]);
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

march_shared_msgs__srv__GetGaitParameters_Response__Sequence *
march_shared_msgs__srv__GetGaitParameters_Response__Sequence__create(size_t size)
{
  march_shared_msgs__srv__GetGaitParameters_Response__Sequence * array = (march_shared_msgs__srv__GetGaitParameters_Response__Sequence *)malloc(sizeof(march_shared_msgs__srv__GetGaitParameters_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = march_shared_msgs__srv__GetGaitParameters_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
march_shared_msgs__srv__GetGaitParameters_Response__Sequence__destroy(march_shared_msgs__srv__GetGaitParameters_Response__Sequence * array)
{
  if (array) {
    march_shared_msgs__srv__GetGaitParameters_Response__Sequence__fini(array);
  }
  free(array);
}
