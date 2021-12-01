// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/MpcJoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/mpc_joint__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/mpc_joint__functions.h"
#include "march_shared_msgs/msg/detail/mpc_joint__struct.h"


// Include directives for member types
// Member `estimation`
// Member `reference`
#include "march_shared_msgs/msg/mpc_state_vectors.h"
// Member `estimation`
// Member `reference`
#include "march_shared_msgs/msg/detail/mpc_state_vectors__rosidl_typesupport_introspection_c.h"
// Member `tuning`
#include "march_shared_msgs/msg/mpc_tuning.h"
// Member `tuning`
#include "march_shared_msgs/msg/detail/mpc_tuning__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__MpcJoint__init(message_memory);
}

void MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_fini_function(void * message_memory)
{
  march_shared_msgs__msg__MpcJoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_member_array[3] = {
  {
    "estimation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcJoint, estimation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reference",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcJoint, reference),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tuning",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcJoint, tuning),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "MpcJoint",  // message name
  3,  // number of fields
  sizeof(march_shared_msgs__msg__MpcJoint),
  MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_member_array,  // message members
  MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_init_function,  // function to initialize message memory (memory has to be allocated)
  MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_type_support_handle = {
  0,
  &MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcJoint)() {
  MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcStateVectors)();
  MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcStateVectors)();
  MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcTuning)();
  if (!MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_type_support_handle.typesupport_identifier) {
    MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MpcJoint__rosidl_typesupport_introspection_c__MpcJoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
