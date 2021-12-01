// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/MpcStateVectors.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_STATE_VECTORS__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_STATE_VECTORS__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/mpc_state_vectors__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_MpcStateVectors_inputs
{
public:
  explicit Init_MpcStateVectors_inputs(::march_shared_msgs::msg::MpcStateVectors & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::MpcStateVectors inputs(::march_shared_msgs::msg::MpcStateVectors::_inputs_type arg)
  {
    msg_.inputs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcStateVectors msg_;
};

class Init_MpcStateVectors_states
{
public:
  Init_MpcStateVectors_states()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MpcStateVectors_inputs states(::march_shared_msgs::msg::MpcStateVectors::_states_type arg)
  {
    msg_.states = std::move(arg);
    return Init_MpcStateVectors_inputs(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcStateVectors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::MpcStateVectors>()
{
  return march_shared_msgs::msg::builder::Init_MpcStateVectors_states();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_STATE_VECTORS__BUILDER_HPP_
