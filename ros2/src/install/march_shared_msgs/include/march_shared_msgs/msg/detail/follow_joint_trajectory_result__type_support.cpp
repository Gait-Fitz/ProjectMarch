// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryResult.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_result__struct.hpp"
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

void FollowJointTrajectoryResult_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) march_shared_msgs::msg::FollowJointTrajectoryResult(_init);
}

void FollowJointTrajectoryResult_fini_function(void * message_memory)
{
  auto typed_message = static_cast<march_shared_msgs::msg::FollowJointTrajectoryResult *>(message_memory);
  typed_message->~FollowJointTrajectoryResult();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FollowJointTrajectoryResult_message_member_array[2] = {
  {
    "error_code",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::FollowJointTrajectoryResult, error_code),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "error_string",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::FollowJointTrajectoryResult, error_string),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FollowJointTrajectoryResult_message_members = {
  "march_shared_msgs::msg",  // message namespace
  "FollowJointTrajectoryResult",  // message name
  2,  // number of fields
  sizeof(march_shared_msgs::msg::FollowJointTrajectoryResult),
  FollowJointTrajectoryResult_message_member_array,  // message members
  FollowJointTrajectoryResult_init_function,  // function to initialize message memory (memory has to be allocated)
  FollowJointTrajectoryResult_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FollowJointTrajectoryResult_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FollowJointTrajectoryResult_message_members,
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
get_message_type_support_handle<march_shared_msgs::msg::FollowJointTrajectoryResult>()
{
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::FollowJointTrajectoryResult_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, march_shared_msgs, msg, FollowJointTrajectoryResult)() {
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::FollowJointTrajectoryResult_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
