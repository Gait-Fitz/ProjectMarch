// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:srv/GetJointNames.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_JOINT_NAMES__BUILDER_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_JOINT_NAMES__BUILDER_HPP_

#include "march_shared_msgs/srv/detail/get_joint_names__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::GetJointNames_Request>()
{
  return ::march_shared_msgs::srv::GetJointNames_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace march_shared_msgs


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_GetJointNames_Response_joint_names
{
public:
  Init_GetJointNames_Response_joint_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::march_shared_msgs::srv::GetJointNames_Response joint_names(::march_shared_msgs::srv::GetJointNames_Response::_joint_names_type arg)
  {
    msg_.joint_names = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::GetJointNames_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::GetJointNames_Response>()
{
  return march_shared_msgs::srv::builder::Init_GetJointNames_Response_joint_names();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_JOINT_NAMES__BUILDER_HPP_
