// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/MpcStateVectors.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/mpc_state_vectors__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/mpc_state_vectors__functions.h"
#include "march_shared_msgs/msg/detail/mpc_state_vectors__struct.h"


// Include directives for member types
// Member `states`
// Member `inputs`
#include "march_shared_msgs/msg/mpc_array.h"
// Member `states`
// Member `inputs`
#include "march_shared_msgs/msg/detail/mpc_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__MpcStateVectors__init(message_memory);
}

void MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_fini_function(void * message_memory)
{
  march_shared_msgs__msg__MpcStateVectors__fini(message_memory);
}

size_t MpcStateVectors__rosidl_typesupport_introspection_c__size_function__MpcArray__states(
  const void * untyped_member)
{
  const march_shared_msgs__msg__MpcArray__Sequence * member =
    (const march_shared_msgs__msg__MpcArray__Sequence *)(untyped_member);
  return member->size;
}

const void * MpcStateVectors__rosidl_typesupport_introspection_c__get_const_function__MpcArray__states(
  const void * untyped_member, size_t index)
{
  const march_shared_msgs__msg__MpcArray__Sequence * member =
    (const march_shared_msgs__msg__MpcArray__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MpcStateVectors__rosidl_typesupport_introspection_c__get_function__MpcArray__states(
  void * untyped_member, size_t index)
{
  march_shared_msgs__msg__MpcArray__Sequence * member =
    (march_shared_msgs__msg__MpcArray__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MpcStateVectors__rosidl_typesupport_introspection_c__resize_function__MpcArray__states(
  void * untyped_member, size_t size)
{
  march_shared_msgs__msg__MpcArray__Sequence * member =
    (march_shared_msgs__msg__MpcArray__Sequence *)(untyped_member);
  march_shared_msgs__msg__MpcArray__Sequence__fini(member);
  return march_shared_msgs__msg__MpcArray__Sequence__init(member, size);
}

size_t MpcStateVectors__rosidl_typesupport_introspection_c__size_function__MpcArray__inputs(
  const void * untyped_member)
{
  const march_shared_msgs__msg__MpcArray__Sequence * member =
    (const march_shared_msgs__msg__MpcArray__Sequence *)(untyped_member);
  return member->size;
}

const void * MpcStateVectors__rosidl_typesupport_introspection_c__get_const_function__MpcArray__inputs(
  const void * untyped_member, size_t index)
{
  const march_shared_msgs__msg__MpcArray__Sequence * member =
    (const march_shared_msgs__msg__MpcArray__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MpcStateVectors__rosidl_typesupport_introspection_c__get_function__MpcArray__inputs(
  void * untyped_member, size_t index)
{
  march_shared_msgs__msg__MpcArray__Sequence * member =
    (march_shared_msgs__msg__MpcArray__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MpcStateVectors__rosidl_typesupport_introspection_c__resize_function__MpcArray__inputs(
  void * untyped_member, size_t size)
{
  march_shared_msgs__msg__MpcArray__Sequence * member =
    (march_shared_msgs__msg__MpcArray__Sequence *)(untyped_member);
  march_shared_msgs__msg__MpcArray__Sequence__fini(member);
  return march_shared_msgs__msg__MpcArray__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_member_array[2] = {
  {
    "states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcStateVectors, states),  // bytes offset in struct
    NULL,  // default value
    MpcStateVectors__rosidl_typesupport_introspection_c__size_function__MpcArray__states,  // size() function pointer
    MpcStateVectors__rosidl_typesupport_introspection_c__get_const_function__MpcArray__states,  // get_const(index) function pointer
    MpcStateVectors__rosidl_typesupport_introspection_c__get_function__MpcArray__states,  // get(index) function pointer
    MpcStateVectors__rosidl_typesupport_introspection_c__resize_function__MpcArray__states  // resize(index) function pointer
  },
  {
    "inputs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__MpcStateVectors, inputs),  // bytes offset in struct
    NULL,  // default value
    MpcStateVectors__rosidl_typesupport_introspection_c__size_function__MpcArray__inputs,  // size() function pointer
    MpcStateVectors__rosidl_typesupport_introspection_c__get_const_function__MpcArray__inputs,  // get_const(index) function pointer
    MpcStateVectors__rosidl_typesupport_introspection_c__get_function__MpcArray__inputs,  // get(index) function pointer
    MpcStateVectors__rosidl_typesupport_introspection_c__resize_function__MpcArray__inputs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "MpcStateVectors",  // message name
  2,  // number of fields
  sizeof(march_shared_msgs__msg__MpcStateVectors),
  MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_member_array,  // message members
  MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_init_function,  // function to initialize message memory (memory has to be allocated)
  MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_type_support_handle = {
  0,
  &MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcStateVectors)() {
  MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcArray)();
  MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, MpcArray)();
  if (!MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_type_support_handle.typesupport_identifier) {
    MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MpcStateVectors__rosidl_typesupport_introspection_c__MpcStateVectors_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
