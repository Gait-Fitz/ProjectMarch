// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryFeedback.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_feedback__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_feedback__functions.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_feedback__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `desired`
// Member `actual`
// Member `error`
#include "trajectory_msgs/msg/joint_trajectory_point.h"
// Member `desired`
// Member `actual`
// Member `error`
#include "trajectory_msgs/msg/detail/joint_trajectory_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__FollowJointTrajectoryFeedback__init(message_memory);
}

void FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_fini_function(void * message_memory)
{
  march_shared_msgs__msg__FollowJointTrajectoryFeedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryFeedback, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryFeedback, joint_names),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "desired",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryFeedback, desired),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actual",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryFeedback, actual),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryFeedback, error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "FollowJointTrajectoryFeedback",  // message name
  5,  // number of fields
  sizeof(march_shared_msgs__msg__FollowJointTrajectoryFeedback),
  FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_member_array,  // message members
  FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_init_function,  // function to initialize message memory (memory has to be allocated)
  FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_type_support_handle = {
  0,
  &FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, FollowJointTrajectoryFeedback)() {
  FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, trajectory_msgs, msg, JointTrajectoryPoint)();
  FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, trajectory_msgs, msg, JointTrajectoryPoint)();
  FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, trajectory_msgs, msg, JointTrajectoryPoint)();
  if (!FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_type_support_handle.typesupport_identifier) {
    FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &FollowJointTrajectoryFeedback__rosidl_typesupport_introspection_c__FollowJointTrajectoryFeedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
