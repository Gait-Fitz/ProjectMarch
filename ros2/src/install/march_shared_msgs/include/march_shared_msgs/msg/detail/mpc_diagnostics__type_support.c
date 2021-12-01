// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/MpcDiagnostics.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/mpc_diagnostics__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/mpc_diagnostics__functions.h"
#include "march_shared_msgs/msg/detail/mpc_diagnostics__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__MpcDiagnostics__init(message_memory);
}

void MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_fini_function(void * message_memory)
{
  march_shared_msgs__msg__MpcDiagnostics__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_message_member_array[6] = {
  {
    "preparation_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcDiagnostics, preparation_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcDiagnostics, feedback_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcDiagnostics, total_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "preparation_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcDiagnostics, preparation_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcDiagnostics, feedback_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cost",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcDiagnostics, cost),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "MpcDiagnostics",  // message name
  6,  // number of fields
  sizeof(march_shared_msgs__msg__MpcDiagnostics),
  MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_message_member_array,  // message members
  MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_init_function,  // function to initialize message memory (memory has to be allocated)
  MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_message_type_support_handle = {
  0,
  &MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcDiagnostics)() {
  if (!MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_message_type_support_handle.typesupport_identifier) {
    MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MpcDiagnostics__rosidl_typesupport_introspection_c__MpcDiagnostics_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
