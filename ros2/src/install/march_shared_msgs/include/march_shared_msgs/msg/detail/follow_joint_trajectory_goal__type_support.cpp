// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryGoal.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_goal__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace march_shared_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FollowJointTrajectoryGoal_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) march_shared_msgs::msg::FollowJointTrajectoryGoal(_init);
}

void FollowJointTrajectoryGoal_fini_function(void * message_memory)
{
  auto typed_message = static_cast<march_shared_msgs::msg::FollowJointTrajectoryGoal *>(message_memory);
  typed_message->~FollowJointTrajectoryGoal();
}

size_t size_function__FollowJointTrajectoryGoal__path_tolerance(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<control_msgs::msg::JointTolerance> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FollowJointTrajectoryGoal__path_tolerance(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<control_msgs::msg::JointTolerance> *>(untyped_member);
  return &member[index];
}

void * get_function__FollowJointTrajectoryGoal__path_tolerance(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<control_msgs::msg::JointTolerance> *>(untyped_member);
  return &member[index];
}

void resize_function__FollowJointTrajectoryGoal__path_tolerance(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<control_msgs::msg::JointTolerance> *>(untyped_member);
  member->resize(size);
}

size_t size_function__FollowJointTrajectoryGoal__goal_tolerance(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<control_msgs::msg::JointTolerance> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FollowJointTrajectoryGoal__goal_tolerance(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<control_msgs::msg::JointTolerance> *>(untyped_member);
  return &member[index];
}

void * get_function__FollowJointTrajectoryGoal__goal_tolerance(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<control_msgs::msg::JointTolerance> *>(untyped_member);
  return &member[index];
}

void resize_function__FollowJointTrajectoryGoal__goal_tolerance(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<control_msgs::msg::JointTolerance> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FollowJointTrajectoryGoal_message_member_array[4] = {
  {
    "trajectory",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<trajectory_msgs::msg::JointTrajectory>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::FollowJointTrajectoryGoal, trajectory),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "path_tolerance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<control_msgs::msg::JointTolerance>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::FollowJointTrajectoryGoal, path_tolerance),  // bytes offset in struct
    nullptr,  // default value
    size_function__FollowJointTrajectoryGoal__path_tolerance,  // size() function pointer
    get_const_function__FollowJointTrajectoryGoal__path_tolerance,  // get_const(index) function pointer
    get_function__FollowJointTrajectoryGoal__path_tolerance,  // get(index) function pointer
    resize_function__FollowJointTrajectoryGoal__path_tolerance  // resize(index) function pointer
  },
  {
    "goal_tolerance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<control_msgs::msg::JointTolerance>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::FollowJointTrajectoryGoal, goal_tolerance),  // bytes offset in struct
    nullptr,  // default value
    size_function__FollowJointTrajectoryGoal__goal_tolerance,  // size() function pointer
    get_const_function__FollowJointTrajectoryGoal__goal_tolerance,  // get_const(index) function pointer
    get_function__FollowJointTrajectoryGoal__goal_tolerance,  // get(index) function pointer
    resize_function__FollowJointTrajectoryGoal__goal_tolerance  // resize(index) function pointer
  },
  {
    "goal_time_tolerance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Duration>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::FollowJointTrajectoryGoal, goal_time_tolerance),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FollowJointTrajectoryGoal_message_members = {
  "march_shared_msgs::msg",  // message namespace
  "FollowJointTrajectoryGoal",  // message name
  4,  // number of fields
  sizeof(march_shared_msgs::msg::FollowJointTrajectoryGoal),
  FollowJointTrajectoryGoal_message_member_array,  // message members
  FollowJointTrajectoryGoal_init_function,  // function to initialize message memory (memory has to be allocated)
  FollowJointTrajectoryGoal_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FollowJointTrajectoryGoal_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FollowJointTrajectoryGoal_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace march_shared_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<march_shared_msgs::msg::FollowJointTrajectoryGoal>()
{
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::FollowJointTrajectoryGoal_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, march_shared_msgs, msg, FollowJointTrajectoryGoal)() {
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::FollowJointTrajectoryGoal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
