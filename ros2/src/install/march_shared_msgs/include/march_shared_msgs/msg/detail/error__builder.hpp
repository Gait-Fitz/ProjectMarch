// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/Error.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__ERROR__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__ERROR__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/error__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_Error_type
{
public:
  explicit Init_Error_type(::march_shared_msgs::msg::Error & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::Error type(::march_shared_msgs::msg::Error::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::Error msg_;
};

class Init_Error_error_message
{
public:
  explicit Init_Error_error_message(::march_shared_msgs::msg::Error & msg)
  : msg_(msg)
  {}
  Init_Error_type error_message(::march_shared_msgs::msg::Error::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_Error_type(msg_);
  }

private:
  ::march_shared_msgs::msg::Error msg_;
};

class Init_Error_header
{
public:
  Init_Error_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Error_error_message header(::march_shared_msgs::msg::Error::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Error_error_message(msg_);
  }

private:
  ::march_shared_msgs::msg::Error msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::Error>()
{
  return march_shared_msgs::msg::builder::Init_Error_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__ERROR__BUILDER_HPP_
