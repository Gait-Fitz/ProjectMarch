// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/CurrentIPDState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_IPD_STATE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_IPD_STATE__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/current_ipd_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_CurrentIPDState_menu_name
{
public:
  explicit Init_CurrentIPDState_menu_name(::march_shared_msgs::msg::CurrentIPDState & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::CurrentIPDState menu_name(::march_shared_msgs::msg::CurrentIPDState::_menu_name_type arg)
  {
    msg_.menu_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentIPDState msg_;
};

class Init_CurrentIPDState_header
{
public:
  Init_CurrentIPDState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CurrentIPDState_menu_name header(::march_shared_msgs::msg::CurrentIPDState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CurrentIPDState_menu_name(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentIPDState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::CurrentIPDState>()
{
  return march_shared_msgs::msg::builder::Init_CurrentIPDState_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_IPD_STATE__BUILDER_HPP_
