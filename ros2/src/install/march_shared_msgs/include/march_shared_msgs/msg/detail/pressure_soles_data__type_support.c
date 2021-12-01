// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/PressureSolesData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/pressure_soles_data__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/pressure_soles_data__functions.h"
#include "march_shared_msgs/msg/detail/pressure_soles_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `left_foot`
// Member `right_foot`
#include "march_shared_msgs/msg/pressure_sole_data.h"
// Member `left_foot`
// Member `right_foot`
#include "march_shared_msgs/msg/detail/pressure_sole_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__PressureSolesData__init(message_memory);
}

void PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_fini_function(void * message_memory)
{
  march_shared_msgs__msg__PressureSolesData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PressureSolesData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_foot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PressureSolesData, left_foot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_foot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PressureSolesData, right_foot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "PressureSolesData",  // message name
  3,  // number of fields
  sizeof(march_shared_msgs__msg__PressureSolesData),
  PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_member_array,  // message members
  PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_init_function,  // function to initialize message memory (memory has to be allocated)
  PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_type_support_handle = {
  0,
  &PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, PressureSolesData)() {
  PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, PressureSoleData)();
  PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, PressureSoleData)();
  if (!PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_type_support_handle.typesupport_identifier) {
    PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PressureSolesData__rosidl_typesupport_introspection_c__PressureSolesData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
