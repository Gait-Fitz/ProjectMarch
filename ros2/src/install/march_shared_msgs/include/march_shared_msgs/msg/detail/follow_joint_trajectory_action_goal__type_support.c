// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionGoal.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_goal__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_goal__functions.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_goal__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `goal_id`
#include "actionlib_msgs/msg/goal_id.h"
// Member `goal_id`
#include "actionlib_msgs/msg/detail/goal_id__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "march_shared_msgs/msg/follow_joint_trajectory_goal.h"
// Member `goal`
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_goal__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__FollowJointTrajectoryActionGoal__init(message_memory);
}

void FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_fini_function(void * message_memory)
{
  march_shared_msgs__msg__FollowJointTrajectoryActionGoal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryActionGoal, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryActionGoal, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryActionGoal, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "FollowJointTrajectoryActionGoal",  // message name
  3,  // number of fields
  sizeof(march_shared_msgs__msg__FollowJointTrajectoryActionGoal),
  FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_member_array,  // message members
  FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_init_function,  // function to initialize message memory (memory has to be allocated)
  FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_type_support_handle = {
  0,
  &FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, FollowJointTrajectoryActionGoal)() {
  FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, actionlib_msgs, msg, GoalID)();
  FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, FollowJointTrajectoryGoal)();
  if (!FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_type_support_handle.typesupport_identifier) {
    FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &FollowJointTrajectoryActionGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryActionGoal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
