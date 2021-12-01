// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/MpcTuning.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_TUNING__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_TUNING__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/mpc_tuning__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_MpcTuning_horizon
{
public:
  explicit Init_MpcTuning_horizon(::march_shared_msgs::msg::MpcTuning & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::MpcTuning horizon(::march_shared_msgs::msg::MpcTuning::_horizon_type arg)
  {
    msg_.horizon = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcTuning msg_;
};

class Init_MpcTuning_r_weights
{
public:
  explicit Init_MpcTuning_r_weights(::march_shared_msgs::msg::MpcTuning & msg)
  : msg_(msg)
  {}
  Init_MpcTuning_horizon r_weights(::march_shared_msgs::msg::MpcTuning::_r_weights_type arg)
  {
    msg_.r_weights = std::move(arg);
    return Init_MpcTuning_horizon(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcTuning msg_;
};

class Init_MpcTuning_q_weights
{
public:
  Init_MpcTuning_q_weights()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MpcTuning_r_weights q_weights(::march_shared_msgs::msg::MpcTuning::_q_weights_type arg)
  {
    msg_.q_weights = std::move(arg);
    return Init_MpcTuning_r_weights(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcTuning msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::MpcTuning>()
{
  return march_shared_msgs::msg::builder::Init_MpcTuning_q_weights();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_TUNING__BUILDER_HPP_
