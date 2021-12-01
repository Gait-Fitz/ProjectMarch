// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/PressureSoleData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/pressure_sole_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_PressureSoleData_arch
{
public:
  explicit Init_PressureSoleData_arch(::march_shared_msgs::msg::PressureSoleData & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::PressureSoleData arch(::march_shared_msgs::msg::PressureSoleData::_arch_type arg)
  {
    msg_.arch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSoleData msg_;
};

class Init_PressureSoleData_met5
{
public:
  explicit Init_PressureSoleData_met5(::march_shared_msgs::msg::PressureSoleData & msg)
  : msg_(msg)
  {}
  Init_PressureSoleData_arch met5(::march_shared_msgs::msg::PressureSoleData::_met5_type arg)
  {
    msg_.met5 = std::move(arg);
    return Init_PressureSoleData_arch(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSoleData msg_;
};

class Init_PressureSoleData_toes
{
public:
  explicit Init_PressureSoleData_toes(::march_shared_msgs::msg::PressureSoleData & msg)
  : msg_(msg)
  {}
  Init_PressureSoleData_met5 toes(::march_shared_msgs::msg::PressureSoleData::_toes_type arg)
  {
    msg_.toes = std::move(arg);
    return Init_PressureSoleData_met5(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSoleData msg_;
};

class Init_PressureSoleData_met3
{
public:
  explicit Init_PressureSoleData_met3(::march_shared_msgs::msg::PressureSoleData & msg)
  : msg_(msg)
  {}
  Init_PressureSoleData_toes met3(::march_shared_msgs::msg::PressureSoleData::_met3_type arg)
  {
    msg_.met3 = std::move(arg);
    return Init_PressureSoleData_toes(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSoleData msg_;
};

class Init_PressureSoleData_hallux
{
public:
  explicit Init_PressureSoleData_hallux(::march_shared_msgs::msg::PressureSoleData & msg)
  : msg_(msg)
  {}
  Init_PressureSoleData_met3 hallux(::march_shared_msgs::msg::PressureSoleData::_hallux_type arg)
  {
    msg_.hallux = std::move(arg);
    return Init_PressureSoleData_met3(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSoleData msg_;
};

class Init_PressureSoleData_met1
{
public:
  explicit Init_PressureSoleData_met1(::march_shared_msgs::msg::PressureSoleData & msg)
  : msg_(msg)
  {}
  Init_PressureSoleData_hallux met1(::march_shared_msgs::msg::PressureSoleData::_met1_type arg)
  {
    msg_.met1 = std::move(arg);
    return Init_PressureSoleData_hallux(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSoleData msg_;
};

class Init_PressureSoleData_heel_left
{
public:
  explicit Init_PressureSoleData_heel_left(::march_shared_msgs::msg::PressureSoleData & msg)
  : msg_(msg)
  {}
  Init_PressureSoleData_met1 heel_left(::march_shared_msgs::msg::PressureSoleData::_heel_left_type arg)
  {
    msg_.heel_left = std::move(arg);
    return Init_PressureSoleData_met1(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSoleData msg_;
};

class Init_PressureSoleData_heel_right
{
public:
  explicit Init_PressureSoleData_heel_right(::march_shared_msgs::msg::PressureSoleData & msg)
  : msg_(msg)
  {}
  Init_PressureSoleData_heel_left heel_right(::march_shared_msgs::msg::PressureSoleData::_heel_right_type arg)
  {
    msg_.heel_right = std::move(arg);
    return Init_PressureSoleData_heel_left(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSoleData msg_;
};

class Init_PressureSoleData_side
{
public:
  Init_PressureSoleData_side()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressureSoleData_heel_right side(::march_shared_msgs::msg::PressureSoleData::_side_type arg)
  {
    msg_.side = std::move(arg);
    return Init_PressureSoleData_heel_right(msg_);
  }

private:
  ::march_shared_msgs::msg::PressureSoleData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::PressureSoleData>()
{
  return march_shared_msgs::msg::builder::Init_PressureSoleData_side();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__BUILDER_HPP_
