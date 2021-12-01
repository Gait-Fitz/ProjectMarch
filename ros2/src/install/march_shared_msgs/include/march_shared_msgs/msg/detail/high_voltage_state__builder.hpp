// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/HighVoltageState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__HIGH_VOLTAGE_STATE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__HIGH_VOLTAGE_STATE__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/high_voltage_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_HighVoltageState_hv4_current
{
public:
  explicit Init_HighVoltageState_hv4_current(::march_shared_msgs::msg::HighVoltageState & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::HighVoltageState hv4_current(::march_shared_msgs::msg::HighVoltageState::_hv4_current_type arg)
  {
    msg_.hv4_current = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::HighVoltageState msg_;
};

class Init_HighVoltageState_hv3_current
{
public:
  explicit Init_HighVoltageState_hv3_current(::march_shared_msgs::msg::HighVoltageState & msg)
  : msg_(msg)
  {}
  Init_HighVoltageState_hv4_current hv3_current(::march_shared_msgs::msg::HighVoltageState::_hv3_current_type arg)
  {
    msg_.hv3_current = std::move(arg);
    return Init_HighVoltageState_hv4_current(msg_);
  }

private:
  ::march_shared_msgs::msg::HighVoltageState msg_;
};

class Init_HighVoltageState_hv2_current
{
public:
  explicit Init_HighVoltageState_hv2_current(::march_shared_msgs::msg::HighVoltageState & msg)
  : msg_(msg)
  {}
  Init_HighVoltageState_hv3_current hv2_current(::march_shared_msgs::msg::HighVoltageState::_hv2_current_type arg)
  {
    msg_.hv2_current = std::move(arg);
    return Init_HighVoltageState_hv3_current(msg_);
  }

private:
  ::march_shared_msgs::msg::HighVoltageState msg_;
};

class Init_HighVoltageState_hv1_current
{
public:
  explicit Init_HighVoltageState_hv1_current(::march_shared_msgs::msg::HighVoltageState & msg)
  : msg_(msg)
  {}
  Init_HighVoltageState_hv2_current hv1_current(::march_shared_msgs::msg::HighVoltageState::_hv1_current_type arg)
  {
    msg_.hv1_current = std::move(arg);
    return Init_HighVoltageState_hv2_current(msg_);
  }

private:
  ::march_shared_msgs::msg::HighVoltageState msg_;
};

class Init_HighVoltageState_total_current
{
public:
  Init_HighVoltageState_total_current()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HighVoltageState_hv1_current total_current(::march_shared_msgs::msg::HighVoltageState::_total_current_type arg)
  {
    msg_.total_current = std::move(arg);
    return Init_HighVoltageState_hv1_current(msg_);
  }

private:
  ::march_shared_msgs::msg::HighVoltageState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::HighVoltageState>()
{
  return march_shared_msgs::msg::builder::Init_HighVoltageState_total_current();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__HIGH_VOLTAGE_STATE__BUILDER_HPP_
