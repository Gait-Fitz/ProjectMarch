// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/MpcDiagnostics.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_DIAGNOSTICS__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_DIAGNOSTICS__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/mpc_diagnostics__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_MpcDiagnostics_cost
{
public:
  explicit Init_MpcDiagnostics_cost(::march_shared_msgs::msg::MpcDiagnostics & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::MpcDiagnostics cost(::march_shared_msgs::msg::MpcDiagnostics::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcDiagnostics msg_;
};

class Init_MpcDiagnostics_feedback_status
{
public:
  explicit Init_MpcDiagnostics_feedback_status(::march_shared_msgs::msg::MpcDiagnostics & msg)
  : msg_(msg)
  {}
  Init_MpcDiagnostics_cost feedback_status(::march_shared_msgs::msg::MpcDiagnostics::_feedback_status_type arg)
  {
    msg_.feedback_status = std::move(arg);
    return Init_MpcDiagnostics_cost(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcDiagnostics msg_;
};

class Init_MpcDiagnostics_preparation_status
{
public:
  explicit Init_MpcDiagnostics_preparation_status(::march_shared_msgs::msg::MpcDiagnostics & msg)
  : msg_(msg)
  {}
  Init_MpcDiagnostics_feedback_status preparation_status(::march_shared_msgs::msg::MpcDiagnostics::_preparation_status_type arg)
  {
    msg_.preparation_status = std::move(arg);
    return Init_MpcDiagnostics_feedback_status(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcDiagnostics msg_;
};

class Init_MpcDiagnostics_total_time
{
public:
  explicit Init_MpcDiagnostics_total_time(::march_shared_msgs::msg::MpcDiagnostics & msg)
  : msg_(msg)
  {}
  Init_MpcDiagnostics_preparation_status total_time(::march_shared_msgs::msg::MpcDiagnostics::_total_time_type arg)
  {
    msg_.total_time = std::move(arg);
    return Init_MpcDiagnostics_preparation_status(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcDiagnostics msg_;
};

class Init_MpcDiagnostics_feedback_time
{
public:
  explicit Init_MpcDiagnostics_feedback_time(::march_shared_msgs::msg::MpcDiagnostics & msg)
  : msg_(msg)
  {}
  Init_MpcDiagnostics_total_time feedback_time(::march_shared_msgs::msg::MpcDiagnostics::_feedback_time_type arg)
  {
    msg_.feedback_time = std::move(arg);
    return Init_MpcDiagnostics_total_time(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcDiagnostics msg_;
};

class Init_MpcDiagnostics_preparation_time
{
public:
  Init_MpcDiagnostics_preparation_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MpcDiagnostics_feedback_time preparation_time(::march_shared_msgs::msg::MpcDiagnostics::_preparation_time_type arg)
  {
    msg_.preparation_time = std::move(arg);
    return Init_MpcDiagnostics_feedback_time(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcDiagnostics msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::MpcDiagnostics>()
{
  return march_shared_msgs::msg::builder::Init_MpcDiagnostics_preparation_time();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_DIAGNOSTICS__BUILDER_HPP_
