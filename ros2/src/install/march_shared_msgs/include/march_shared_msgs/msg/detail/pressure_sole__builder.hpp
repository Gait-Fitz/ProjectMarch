// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/PressureSole.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/pressure_sole__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_PressureSole_cop_right
{
public:
  explicit Init_PressureSole_cop_right(::march_shared_msgs::msg::PressureSole & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::PressureSole cop_right(::march_shared_msgs::msg::PressureSole::_cop_right_type arg)
  {
    msg_.cop_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSole msg_;
};

class Init_PressureSole_cop_left
{
public:
  explicit Init_PressureSole_cop_left(::march_shared_msgs::msg::PressureSole & msg)
  : msg_(msg)
  {}
  Init_PressureSole_cop_right cop_left(::march_shared_msgs::msg::PressureSole::_cop_left_type arg)
  {
    msg_.cop_left = std::move(arg);
    return Init_PressureSole_cop_right(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSole msg_;
};

class Init_PressureSole_pressure_right
{
public:
  explicit Init_PressureSole_pressure_right(::march_shared_msgs::msg::PressureSole & msg)
  : msg_(msg)
  {}
  Init_PressureSole_cop_left pressure_right(::march_shared_msgs::msg::PressureSole::_pressure_right_type arg)
  {
    msg_.pressure_right = std::move(arg);
    return Init_PressureSole_cop_left(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSole msg_;
};

class Init_PressureSole_pressure_left
{
public:
  explicit Init_PressureSole_pressure_left(::march_shared_msgs::msg::PressureSole & msg)
  : msg_(msg)
  {}
  Init_PressureSole_pressure_right pressure_left(::march_shared_msgs::msg::PressureSole::_pressure_left_type arg)
  {
    msg_.pressure_left = std::move(arg);
    return Init_PressureSole_pressure_right(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSole msg_;
};

class Init_PressureSole_total_force_right
{
public:
  explicit Init_PressureSole_total_force_right(::march_shared_msgs::msg::PressureSole & msg)
  : msg_(msg)
  {}
  Init_PressureSole_pressure_left total_force_right(::march_shared_msgs::msg::PressureSole::_total_force_right_type arg)
  {
    msg_.total_force_right = std::move(arg);
    return Init_PressureSole_pressure_left(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSole msg_;
};

class Init_PressureSole_total_force_left
{
public:
  explicit Init_PressureSole_total_force_left(::march_shared_msgs::msg::PressureSole & msg)
  : msg_(msg)
  {}
  Init_PressureSole_total_force_right total_force_left(::march_shared_msgs::msg::PressureSole::_total_force_left_type arg)
  {
    msg_.total_force_left = std::move(arg);
    return Init_PressureSole_total_force_right(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSole msg_;
};

class Init_PressureSole_pressure_soles_time
{
public:
  explicit Init_PressureSole_pressure_soles_time(::march_shared_msgs::msg::PressureSole & msg)
  : msg_(msg)
  {}
  Init_PressureSole_total_force_left pressure_soles_time(::march_shared_msgs::msg::PressureSole::_pressure_soles_time_type arg)
  {
    msg_.pressure_soles_time = std::move(arg);
    return Init_PressureSole_total_force_left(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSole msg_;
};

class Init_PressureSole_header
{
public:
  Init_PressureSole_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressureSole_pressure_soles_time header(::march_shared_msgs::msg::PressureSole::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PressureSole_pressure_soles_time(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSole msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::PressureSole>()
{
  return march_shared_msgs::msg::builder::Init_PressureSole_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__BUILDER_HPP_
