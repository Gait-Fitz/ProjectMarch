// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/MpcMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/mpc_msg__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/mpc_msg__functions.h"
#include "march_shared_msgs/msg/detail/mpc_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `diagnostics`
#include "march_shared_msgs/msg/mpc_diagnostics.h"
// Member `diagnostics`
#include "march_shared_msgs/msg/detail/mpc_diagnostics__rosidl_typesupport_introspection_c.h"
// Member `joint`
#include "march_shared_msgs/msg/mpc_joint.h"
// Member `joint`
#include "march_shared_msgs/msg/detail/mpc_joint__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__MpcMsg__init(message_memory);
}

void MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_fini_function(void * message_memory)
{
  march_shared_msgs__msg__MpcMsg__fini(message_memory);
}

size_t MpcMsg__rosidl_typesupport_introspection_c__size_function__MpcJoint__joint(
  const void * untyped_member)
{
  const march_shared_msgs__msg__MpcJoint__Sequence * member =
    (const march_shared_msgs__msg__MpcJoint__Sequence *)(untyped_member);
  return member->size;
}

const void * MpcMsg__rosidl_typesupport_introspection_c__get_const_function__MpcJoint__joint(
  const void * untyped_member, size_t index)
{
  const march_shared_msgs__msg__MpcJoint__Sequence * member =
    (const march_shared_msgs__msg__MpcJoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MpcMsg__rosidl_typesupport_introspection_c__get_function__MpcJoint__joint(
  void * untyped_member, size_t index)
{
  march_shared_msgs__msg__MpcJoint__Sequence * member =
    (march_shared_msgs__msg__MpcJoint__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MpcMsg__rosidl_typesupport_introspection_c__resize_function__MpcJoint__joint(
  void * untyped_member, size_t size)
{
  march_shared_msgs__msg__MpcJoint__Sequence * member =
    (march_shared_msgs__msg__MpcJoint__Sequence *)(untyped_member);
  march_shared_msgs__msg__MpcJoint__Sequence__fini(member);
  return march_shared_msgs__msg__MpcJoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "diagnostics",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcMsg, diagnostics),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcMsg, joint),  // bytes offset in struct
    NULL,  // default value
    MpcMsg__rosidl_typesupport_introspection_c__size_function__MpcJoint__joint,  // size() function pointer
    MpcMsg__rosidl_typesupport_introspection_c__get_const_function__MpcJoint__joint,  // get_const(index) function pointer
    MpcMsg__rosidl_typesupport_introspection_c__get_function__MpcJoint__joint,  // get(index) function pointer
    MpcMsg__rosidl_typesupport_introspection_c__resize_function__MpcJoint__joint  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "MpcMsg",  // message name
  3,  // number of fields
  sizeof(march_shared_msgs__msg__MpcMsg),
  MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_member_array,  // message members
  MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_type_support_handle = {
  0,
  &MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcMsg)() {
  MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcDiagnostics)();
  MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcJoint)();
  if (!MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_type_support_handle.typesupport_identifier) {
    MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MpcMsg__rosidl_typesupport_introspection_c__MpcMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
