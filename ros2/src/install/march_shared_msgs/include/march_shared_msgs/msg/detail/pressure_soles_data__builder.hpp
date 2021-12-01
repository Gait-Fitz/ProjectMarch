// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/PressureSolesData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/pressure_soles_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_PressureSolesData_right_foot
{
public:
  explicit Init_PressureSolesData_right_foot(::march_shared_msgs::msg::PressureSolesData & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::PressureSolesData right_foot(::march_shared_msgs::msg::PressureSolesData::_right_foot_type arg)
  {
    msg_.right_foot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSolesData msg_;
};

class Init_PressureSolesData_left_foot
{
public:
  explicit Init_PressureSolesData_left_foot(::march_shared_msgs::msg::PressureSolesData & msg)
  : msg_(msg)
  {}
  Init_PressureSolesData_right_foot left_foot(::march_shared_msgs::msg::PressureSolesData::_left_foot_type arg)
  {
    msg_.left_foot = std::move(arg);
    return Init_PressureSolesData_right_foot(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSolesData msg_;
};

class Init_PressureSolesData_header
{
public:
  Init_PressureSolesData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressureSolesData_left_foot header(::march_shared_msgs::msg::PressureSolesData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PressureSolesData_left_foot(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSolesData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::PressureSolesData>()
{
  return march_shared_msgs::msg::builder::Init_PressureSolesData_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__BUILDER_HPP_
