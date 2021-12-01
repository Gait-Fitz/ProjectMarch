// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/CurrentGait.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/current_gait__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/current_gait__functions.h"
#include "march_shared_msgs/msg/detail/current_gait__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `gait`
// Member `subgait`
// Member `version`
// Member `gait_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `duration`
#include "builtin_interfaces/msg/duration.h"
// Member `duration`
#include "builtin_interfaces/msg/detail/duration__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__CurrentGait__init(message_memory);
}

void CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_fini_function(void * message_memory)
{
  march_shared_msgs__msg__CurrentGait__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__CurrentGait, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gait",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__CurrentGait, gait),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "subgait",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__CurrentGait, subgait),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__CurrentGait, version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__CurrentGait, duration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gait_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__CurrentGait, gait_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "CurrentGait",  // message name
  6,  // number of fields
  sizeof(march_shared_msgs__msg__CurrentGait),
  CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_member_array,  // message members
  CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_init_function,  // function to initialize message memory (memory has to be allocated)
  CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_type_support_handle = {
  0,
  &CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, CurrentGait)() {
  CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Duration)();
  if (!CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_type_support_handle.typesupport_identifier) {
    CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CurrentGait__rosidl_typesupport_introspection_c__CurrentGait_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
