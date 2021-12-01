// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/GaitParameters.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/gait_parameters__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_GaitParameters_side_step_parameter
{
public:
  explicit Init_GaitParameters_side_step_parameter(::march_shared_msgs::msg::GaitParameters & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::GaitParameters side_step_parameter(::march_shared_msgs::msg::GaitParameters::_side_step_parameter_type arg)
  {
    msg_.side_step_parameter = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::GaitParameters msg_;
};

class Init_GaitParameters_second_parameter
{
public:
  explicit Init_GaitParameters_second_parameter(::march_shared_msgs::msg::GaitParameters & msg)
  : msg_(msg)
  {}
  Init_GaitParameters_side_step_parameter second_parameter(::march_shared_msgs::msg::GaitParameters::_second_parameter_type arg)
  {
    msg_.second_parameter = std::move(arg);
    return Init_GaitParameters_side_step_parameter(msg_);
  }

private:
  ::march_shared_msgs::msg::GaitParameters msg_;
};

class Init_GaitParameters_first_parameter
{
public:
  Init_GaitParameters_first_parameter()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GaitParameters_second_parameter first_parameter(::march_shared_msgs::msg::GaitParameters::_first_parameter_type arg)
  {
    msg_.first_parameter = std::move(arg);
    return Init_GaitParameters_second_parameter(msg_);
  }

private:
  ::march_shared_msgs::msg::GaitParameters msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::GaitParameters>()
{
  return march_shared_msgs::msg::builder::Init_GaitParameters_first_parameter();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__BUILDER_HPP_
