// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/MpcMsg.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_MSG__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_MSG__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/mpc_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_MpcMsg_joint
{
public:
  explicit Init_MpcMsg_joint(::march_shared_msgs::msg::MpcMsg & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::MpcMsg joint(::march_shared_msgs::msg::MpcMsg::_joint_type arg)
  {
    msg_.joint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcMsg msg_;
};

class Init_MpcMsg_diagnostics
{
public:
  explicit Init_MpcMsg_diagnostics(::march_shared_msgs::msg::MpcMsg & msg)
  : msg_(msg)
  {}
  Init_MpcMsg_joint diagnostics(::march_shared_msgs::msg::MpcMsg::_diagnostics_type arg)
  {
    msg_.diagnostics = std::move(arg);
    return Init_MpcMsg_joint(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcMsg msg_;
};

class Init_MpcMsg_header
{
public:
  Init_MpcMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MpcMsg_diagnostics header(::march_shared_msgs::msg::MpcMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MpcMsg_diagnostics(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::MpcMsg>()
{
  return march_shared_msgs::msg::builder::Init_MpcMsg_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_MSG__BUILDER_HPP_
