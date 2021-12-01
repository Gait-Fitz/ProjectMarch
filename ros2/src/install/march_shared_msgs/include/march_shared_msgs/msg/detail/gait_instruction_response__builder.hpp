// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/GaitInstructionResponse.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION_RESPONSE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION_RESPONSE__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/gait_instruction_response__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_GaitInstructionResponse_result
{
public:
  Init_GaitInstructionResponse_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::march_shared_msgs::msg::GaitInstructionResponse result(::march_shared_msgs::msg::GaitInstructionResponse::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::GaitInstructionResponse msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::GaitInstructionResponse>()
{
  return march_shared_msgs::msg::builder::Init_GaitInstructionResponse_result();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION_RESPONSE__BUILDER_HPP_
