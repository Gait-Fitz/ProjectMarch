// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionResult.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_result__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_result__functions.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_result__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `status`
#include "actionlib_msgs/msg/goal_status.h"
// Member `status`
#include "actionlib_msgs/msg/detail/goal_status__rosidl_typesupport_introspection_c.h"
// Member `result`
#include "march_shared_msgs/msg/follow_joint_trajectory_result.h"
// Member `result`
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_result__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__FollowJointTrajectoryActionResult__init(message_memory);
}

void FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_fini_function(void * message_memory)
{
  march_shared_msgs__msg__FollowJointTrajectoryActionResult__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryActionResult, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryActionResult, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryActionResult, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "FollowJointTrajectoryActionResult",  // message name
  3,  // number of fields
  sizeof(march_shared_msgs__msg__FollowJointTrajectoryActionResult),
  FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_member_array,  // message members
  FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_init_function,  // function to initialize message memory (memory has to be allocated)
  FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_type_support_handle = {
  0,
  &FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, FollowJointTrajectoryActionResult)() {
  FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, actionlib_msgs, msg, GoalStatus)();
  FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, FollowJointTrajectoryResult)();
  if (!FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_type_support_handle.typesupport_identifier) {
    FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &FollowJointTrajectoryActionResult__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionResult_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
