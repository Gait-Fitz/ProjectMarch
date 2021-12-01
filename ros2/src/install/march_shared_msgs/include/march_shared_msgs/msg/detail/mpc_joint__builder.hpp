// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/MpcJoint.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/mpc_joint__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_MpcJoint_tuning
{
public:
  explicit Init_MpcJoint_tuning(::march_shared_msgs::msg::MpcJoint & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::MpcJoint tuning(::march_shared_msgs::msg::MpcJoint::_tuning_type arg)
  {
    msg_.tuning = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcJoint msg_;
};

class Init_MpcJoint_reference
{
public:
  explicit Init_MpcJoint_reference(::march_shared_msgs::msg::MpcJoint & msg)
  : msg_(msg)
  {}
  Init_MpcJoint_tuning reference(::march_shared_msgs::msg::MpcJoint::_reference_type arg)
  {
    msg_.reference = std::move(arg);
    return Init_MpcJoint_tuning(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcJoint msg_;
};

class Init_MpcJoint_estimation
{
public:
  Init_MpcJoint_estimation()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MpcJoint_reference estimation(::march_shared_msgs::msg::MpcJoint::_estimation_type arg)
  {
    msg_.estimation = std::move(arg);
    return Init_MpcJoint_reference(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcJoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::MpcJoint>()
{
  return march_shared_msgs::msg::builder::Init_MpcJoint_estimation();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__BUILDER_HPP_
