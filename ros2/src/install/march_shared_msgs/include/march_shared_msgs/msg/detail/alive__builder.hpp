// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/Alive.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__ALIVE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__ALIVE__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/alive__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_Alive_id
{
public:
  explicit Init_Alive_id(::march_shared_msgs::msg::Alive & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::Alive id(::march_shared_msgs::msg::Alive::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::Alive msg_;
};

class Init_Alive_stamp
{
public:
  Init_Alive_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Alive_id stamp(::march_shared_msgs::msg::Alive::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Alive_id(msg_);
  }

private:
  ::march_shared_msgs::msg::Alive msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::Alive>()
{
  return march_shared_msgs::msg::builder::Init_Alive_stamp();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__ALIVE__BUILDER_HPP_
