// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/GaitInstruction.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/gait_instruction__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_GaitInstruction_id
{
public:
  explicit Init_GaitInstruction_id(::march_shared_msgs::msg::GaitInstruction & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::GaitInstruction id(::march_shared_msgs::msg::GaitInstruction::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::GaitInstruction msg_;
};

class Init_GaitInstruction_gait_name
{
public:
  explicit Init_GaitInstruction_gait_name(::march_shared_msgs::msg::GaitInstruction & msg)
  : msg_(msg)
  {}
  Init_GaitInstruction_id gait_name(::march_shared_msgs::msg::GaitInstruction::_gait_name_type arg)
  {
    msg_.gait_name = std::move(arg);
    return Init_GaitInstruction_id(msg_);
  }

private:
  ::march_shared_msgs::msg::GaitInstruction msg_;
};

class Init_GaitInstruction_type
{
public:
  explicit Init_GaitInstruction_type(::march_shared_msgs::msg::GaitInstruction & msg)
  : msg_(msg)
  {}
  Init_GaitInstruction_gait_name type(::march_shared_msgs::msg::GaitInstruction::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_GaitInstruction_gait_name(msg_);
  }

private:
  ::march_shared_msgs::msg::GaitInstruction msg_;
};

class Init_GaitInstruction_header
{
public:
  Init_GaitInstruction_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GaitInstruction_type header(::march_shared_msgs::msg::GaitInstruction::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GaitInstruction_type(msg_);
  }

private:
  ::march_shared_msgs::msg::GaitInstruction msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::GaitInstruction>()
{
  return march_shared_msgs::msg::builder::Init_GaitInstruction_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION__BUILDER_HPP_
