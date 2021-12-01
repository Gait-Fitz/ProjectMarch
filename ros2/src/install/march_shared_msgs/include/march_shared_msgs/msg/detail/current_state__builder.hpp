// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/CurrentState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_STATE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_STATE__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/current_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_CurrentState_state_type
{
public:
  explicit Init_CurrentState_state_type(::march_shared_msgs::msg::CurrentState & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::CurrentState state_type(::march_shared_msgs::msg::CurrentState::_state_type_type arg)
  {
    msg_.state_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentState msg_;
};

class Init_CurrentState_state
{
public:
  explicit Init_CurrentState_state(::march_shared_msgs::msg::CurrentState & msg)
  : msg_(msg)
  {}
  Init_CurrentState_state_type state(::march_shared_msgs::msg::CurrentState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_CurrentState_state_type(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentState msg_;
};

class Init_CurrentState_header
{
public:
  Init_CurrentState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CurrentState_state header(::march_shared_msgs::msg::CurrentState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CurrentState_state(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::CurrentState>()
{
  return march_shared_msgs::msg::builder::Init_CurrentState_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_STATE__BUILDER_HPP_
