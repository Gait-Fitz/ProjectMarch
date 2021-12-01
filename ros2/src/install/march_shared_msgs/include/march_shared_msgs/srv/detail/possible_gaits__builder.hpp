// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:srv/PossibleGaits.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__POSSIBLE_GAITS__BUILDER_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__POSSIBLE_GAITS__BUILDER_HPP_

#include "march_shared_msgs/srv/detail/possible_gaits__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::PossibleGaits_Request>()
{
  return ::march_shared_msgs::srv::PossibleGaits_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace march_shared_msgs


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_PossibleGaits_Response_gaits
{
public:
  Init_PossibleGaits_Response_gaits()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::march_shared_msgs::srv::PossibleGaits_Response gaits(::march_shared_msgs::srv::PossibleGaits_Response::_gaits_type arg)
  {
    msg_.gaits = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::PossibleGaits_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::PossibleGaits_Response>()
{
  return march_shared_msgs::srv::builder::Init_PossibleGaits_Response_gaits();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__POSSIBLE_GAITS__BUILDER_HPP_
