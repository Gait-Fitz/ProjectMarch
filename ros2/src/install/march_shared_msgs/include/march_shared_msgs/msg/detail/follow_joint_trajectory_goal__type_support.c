// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryGoal.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_goal__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_goal__functions.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_goal__struct.h"


// Include directives for member types
// Member `trajectory`
#include "trajectory_msgs/msg/joint_trajectory.h"
// Member `trajectory`
#include "trajectory_msgs/msg/detail/joint_trajectory__rosidl_typesupport_introspection_c.h"
// Member `path_tolerance`
// Member `goal_tolerance`
#include "control_msgs/msg/joint_tolerance.h"
// Member `path_tolerance`
// Member `goal_tolerance`
#include "control_msgs/msg/detail/joint_tolerance__rosidl_typesupport_introspection_c.h"
// Member `goal_time_tolerance`
#include "builtin_interfaces/msg/duration.h"
// Member `goal_time_tolerance`
#include "builtin_interfaces/msg/detail/duration__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__FollowJointTrajectoryGoal__init(message_memory);
}

void FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_fini_function(void * message_memory)
{
  march_shared_msgs__msg__FollowJointTrajectoryGoal__fini(message_memory);
}

size_t FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__size_function__JointTolerance__path_tolerance(
  const void * untyped_member)
{
  const control_msgs__msg__JointTolerance__Sequence * member =
    (const control_msgs__msg__JointTolerance__Sequence *)(untyped_member);
  return member->size;
}

const void * FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__get_const_function__JointTolerance__path_tolerance(
  const void * untyped_member, size_t index)
{
  const control_msgs__msg__JointTolerance__Sequence * member =
    (const control_msgs__msg__JointTolerance__Sequence *)(untyped_member);
  return &member->data[index];
}

void * FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__get_function__JointTolerance__path_tolerance(
  void * untyped_member, size_t index)
{
  control_msgs__msg__JointTolerance__Sequence * member =
    (control_msgs__msg__JointTolerance__Sequence *)(untyped_member);
  return &member->data[index];
}

bool FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__resize_function__JointTolerance__path_tolerance(
  void * untyped_member, size_t size)
{
  control_msgs__msg__JointTolerance__Sequence * member =
    (control_msgs__msg__JointTolerance__Sequence *)(untyped_member);
  control_msgs__msg__JointTolerance__Sequence__fini(member);
  return control_msgs__msg__JointTolerance__Sequence__init(member, size);
}

size_t FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__size_function__JointTolerance__goal_tolerance(
  const void * untyped_member)
{
  const control_msgs__msg__JointTolerance__Sequence * member =
    (const control_msgs__msg__JointTolerance__Sequence *)(untyped_member);
  return member->size;
}

const void * FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__get_const_function__JointTolerance__goal_tolerance(
  const void * untyped_member, size_t index)
{
  const control_msgs__msg__JointTolerance__Sequence * member =
    (const control_msgs__msg__JointTolerance__Sequence *)(untyped_member);
  return &member->data[index];
}

void * FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__get_function__JointTolerance__goal_tolerance(
  void * untyped_member, size_t index)
{
  control_msgs__msg__JointTolerance__Sequence * member =
    (control_msgs__msg__JointTolerance__Sequence *)(untyped_member);
  return &member->data[index];
}

bool FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__resize_function__JointTolerance__goal_tolerance(
  void * untyped_member, size_t size)
{
  control_msgs__msg__JointTolerance__Sequence * member =
    (control_msgs__msg__JointTolerance__Sequence *)(untyped_member);
  control_msgs__msg__JointTolerance__Sequence__fini(member);
  return control_msgs__msg__JointTolerance__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_member_array[4] = {
  {
    "trajectory",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryGoal, trajectory),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "path_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryGoal, path_tolerance),  // bytes offset in struct
    NULL,  // default value
    FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__size_function__JointTolerance__path_tolerance,  // size() function pointer
    FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__get_const_function__JointTolerance__path_tolerance,  // get_const(index) function pointer
    FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__get_function__JointTolerance__path_tolerance,  // get(index) function pointer
    FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__resize_function__JointTolerance__path_tolerance  // resize(index) function pointer
  },
  {
    "goal_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryGoal, goal_tolerance),  // bytes offset in struct
    NULL,  // default value
    FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__size_function__JointTolerance__goal_tolerance,  // size() function pointer
    FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__get_const_function__JointTolerance__goal_tolerance,  // get_const(index) function pointer
    FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__get_function__JointTolerance__goal_tolerance,  // get(index) function pointer
    FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__resize_function__JointTolerance__goal_tolerance  // resize(index) function pointer
  },
  {
    "goal_time_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__FollowJointTrajectoryGoal, goal_time_tolerance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "FollowJointTrajectoryGoal",  // message name
  4,  // number of fields
  sizeof(march_shared_msgs__msg__FollowJointTrajectoryGoal),
  FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_member_array,  // message members
  FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_init_function,  // function to initialize message memory (memory has to be allocated)
  FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_type_support_handle = {
  0,
  &FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, FollowJointTrajectoryGoal)() {
  FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, trajectory_msgs, msg, JointTrajectory)();
  FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_msgs, msg, JointTolerance)();
  FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_msgs, msg, JointTolerance)();
  FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Duration)();
  if (!FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_type_support_handle.typesupport_identifier) {
    FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &FollowJointTrajectoryGoal__rosidl_typesupport_introspection_c__FollowJointTrajectoryGoal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
