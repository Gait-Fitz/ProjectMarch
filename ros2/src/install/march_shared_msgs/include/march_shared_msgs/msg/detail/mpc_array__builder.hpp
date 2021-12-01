// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/MpcArray.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_ARRAY__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_ARRAY__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/mpc_array__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_MpcArray_array
{
public:
  Init_MpcArray_array()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::march_shared_msgs::msg::MpcArray array(::march_shared_msgs::msg::MpcArray::_array_type arg)
  {
    msg_.array = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::MpcArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::MpcArray>()
{
  return march_shared_msgs::msg::builder::Init_MpcArray_array();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_ARRAY__BUILDER_HPP_
