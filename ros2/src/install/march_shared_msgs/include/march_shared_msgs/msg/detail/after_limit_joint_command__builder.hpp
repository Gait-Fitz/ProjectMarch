// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/AfterLimitJointCommand.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__AFTER_LIMIT_JOINT_COMMAND__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__AFTER_LIMIT_JOINT_COMMAND__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/after_limit_joint_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_AfterLimitJointCommand_effort_command
{
public:
  explicit Init_AfterLimitJointCommand_effort_command(::march_shared_msgs::msg::AfterLimitJointCommand & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::AfterLimitJointCommand effort_command(::march_shared_msgs::msg::AfterLimitJointCommand::_effort_command_type arg)
  {
    msg_.effort_command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::AfterLimitJointCommand msg_;
};

class Init_AfterLimitJointCommand_position_command
{
public:
  explicit Init_AfterLimitJointCommand_position_command(::march_shared_msgs::msg::AfterLimitJointCommand & msg)
  : msg_(msg)
  {}
  Init_AfterLimitJointCommand_effort_command position_command(::march_shared_msgs::msg::AfterLimitJointCommand::_position_command_type arg)
  {
    msg_.position_command = std::move(arg);
    return Init_AfterLimitJointCommand_effort_command(msg_);
  }

private:
  ::march_shared_msgs::msg::AfterLimitJointCommand msg_;
};

class Init_AfterLimitJointCommand_name
{
public:
  explicit Init_AfterLimitJointCommand_name(::march_shared_msgs::msg::AfterLimitJointCommand & msg)
  : msg_(msg)
  {}
  Init_AfterLimitJointCommand_position_command name(::march_shared_msgs::msg::AfterLimitJointCommand::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_AfterLimitJointCommand_position_command(msg_);
  }

private:
  ::march_shared_msgs::msg::AfterLimitJointCommand msg_;
};

class Init_AfterLimitJointCommand_header
{
public:
  Init_AfterLimitJointCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AfterLimitJointCommand_name header(::march_shared_msgs::msg::AfterLimitJointCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AfterLimitJointCommand_name(msg_);
  }

private:
  ::march_shared_msgs::msg::AfterLimitJointCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::AfterLimitJointCommand>()
{
  return march_shared_msgs::msg::builder::Init_AfterLimitJointCommand_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__AFTER_LIMIT_JOINT_COMMAND__BUILDER_HPP_
