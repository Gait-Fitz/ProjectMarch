// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/PowerDistributionBoardData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/power_distribution_board_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_PowerDistributionBoardData_battery_state
{
public:
  explicit Init_PowerDistributionBoardData_battery_state(::march_shared_msgs::msg::PowerDistributionBoardData & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::PowerDistributionBoardData battery_state(::march_shared_msgs::msg::PowerDistributionBoardData::_battery_state_type arg)
  {
    msg_.battery_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::PowerDistributionBoardData msg_;
};

class Init_PowerDistributionBoardData_lv_state
{
public:
  explicit Init_PowerDistributionBoardData_lv_state(::march_shared_msgs::msg::PowerDistributionBoardData & msg)
  : msg_(msg)
  {}
  Init_PowerDistributionBoardData_battery_state lv_state(::march_shared_msgs::msg::PowerDistributionBoardData::_lv_state_type arg)
  {
    msg_.lv_state = std::move(arg);
    return Init_PowerDistributionBoardData_battery_state(msg_);
  }

private:
  ::march_shared_msgs::msg::PowerDistributionBoardData msg_;
};

class Init_PowerDistributionBoardData_stop_button_state
{
public:
  explicit Init_PowerDistributionBoardData_stop_button_state(::march_shared_msgs::msg::PowerDistributionBoardData & msg)
  : msg_(msg)
  {}
  Init_PowerDistributionBoardData_lv_state stop_button_state(::march_shared_msgs::msg::PowerDistributionBoardData::_stop_button_state_type arg)
  {
    msg_.stop_button_state = std::move(arg);
    return Init_PowerDistributionBoardData_lv_state(msg_);
  }

private:
  ::march_shared_msgs::msg::PowerDistributionBoardData msg_;
};

class Init_PowerDistributionBoardData_hv_state
{
public:
  explicit Init_PowerDistributionBoardData_hv_state(::march_shared_msgs::msg::PowerDistributionBoardData & msg)
  : msg_(msg)
  {}
  Init_PowerDistributionBoardData_stop_button_state hv_state(::march_shared_msgs::msg::PowerDistributionBoardData::_hv_state_type arg)
  {
    msg_.hv_state = std::move(arg);
    return Init_PowerDistributionBoardData_stop_button_state(msg_);
  }

private:
  ::march_shared_msgs::msg::PowerDistributionBoardData msg_;
};

class Init_PowerDistributionBoardData_pdb_current
{
public:
  explicit Init_PowerDistributionBoardData_pdb_current(::march_shared_msgs::msg::PowerDistributionBoardData & msg)
  : msg_(msg)
  {}
  Init_PowerDistributionBoardData_hv_state pdb_current(::march_shared_msgs::msg::PowerDistributionBoardData::_pdb_current_type arg)
  {
    msg_.pdb_current = std::move(arg);
    return Init_PowerDistributionBoardData_hv_state(msg_);
  }

private:
  ::march_shared_msgs::msg::PowerDistributionBoardData msg_;
};

class Init_PowerDistributionBoardData_emergency_button_state
{
public:
  explicit Init_PowerDistributionBoardData_emergency_button_state(::march_shared_msgs::msg::PowerDistributionBoardData & msg)
  : msg_(msg)
  {}
  Init_PowerDistributionBoardData_pdb_current emergency_button_state(::march_shared_msgs::msg::PowerDistributionBoardData::_emergency_button_state_type arg)
  {
    msg_.emergency_button_state = std::move(arg);
    return Init_PowerDistributionBoardData_pdb_current(msg_);
  }

private:
  ::march_shared_msgs::msg::PowerDistributionBoardData msg_;
};

class Init_PowerDistributionBoardData_header
{
public:
  Init_PowerDistributionBoardData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PowerDistributionBoardData_emergency_button_state header(::march_shared_msgs::msg::PowerDistributionBoardData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PowerDistributionBoardData_emergency_button_state(msg_);
  }

private:
  ::march_shared_msgs::msg::PowerDistributionBoardData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::PowerDistributionBoardData>()
{
  return march_shared_msgs::msg::builder::Init_PowerDistributionBoardData_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__BUILDER_HPP_
