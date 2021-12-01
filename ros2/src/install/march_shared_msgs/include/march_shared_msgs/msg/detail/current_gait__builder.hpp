// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/CurrentGait.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_GAIT__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_GAIT__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/current_gait__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_CurrentGait_gait_type
{
public:
  explicit Init_CurrentGait_gait_type(::march_shared_msgs::msg::CurrentGait & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::CurrentGait gait_type(::march_shared_msgs::msg::CurrentGait::_gait_type_type arg)
  {
    msg_.gait_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentGait msg_;
};

class Init_CurrentGait_duration
{
public:
  explicit Init_CurrentGait_duration(::march_shared_msgs::msg::CurrentGait & msg)
  : msg_(msg)
  {}
  Init_CurrentGait_gait_type duration(::march_shared_msgs::msg::CurrentGait::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_CurrentGait_gait_type(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentGait msg_;
};

class Init_CurrentGait_version
{
public:
  explicit Init_CurrentGait_version(::march_shared_msgs::msg::CurrentGait & msg)
  : msg_(msg)
  {}
  Init_CurrentGait_duration version(::march_shared_msgs::msg::CurrentGait::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_CurrentGait_duration(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentGait msg_;
};

class Init_CurrentGait_subgait
{
public:
  explicit Init_CurrentGait_subgait(::march_shared_msgs::msg::CurrentGait & msg)
  : msg_(msg)
  {}
  Init_CurrentGait_version subgait(::march_shared_msgs::msg::CurrentGait::_subgait_type arg)
  {
    msg_.subgait = std::move(arg);
    return Init_CurrentGait_version(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentGait msg_;
};

class Init_CurrentGait_gait
{
public:
  explicit Init_CurrentGait_gait(::march_shared_msgs::msg::CurrentGait & msg)
  : msg_(msg)
  {}
  Init_CurrentGait_subgait gait(::march_shared_msgs::msg::CurrentGait::_gait_type arg)
  {
    msg_.gait = std::move(arg);
    return Init_CurrentGait_subgait(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentGait msg_;
};

class Init_CurrentGait_header
{
public:
  Init_CurrentGait_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CurrentGait_gait header(::march_shared_msgs::msg::CurrentGait::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CurrentGait_gait(msg_);
  }

private:
  ::march_shared_msgs::msg::CurrentGait msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::CurrentGait>()
{
  return march_shared_msgs::msg::builder::Init_CurrentGait_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_GAIT__BUILDER_HPP_
