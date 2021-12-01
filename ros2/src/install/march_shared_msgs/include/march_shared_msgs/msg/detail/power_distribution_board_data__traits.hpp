// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:msg/PowerDistributionBoardData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__TRAITS_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__TRAITS_HPP_

#include "march_shared_msgs/msg/detail/power_distribution_board_data__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'hv_state'
#include "march_shared_msgs/msg/detail/high_voltage_state__traits.hpp"
// Member 'lv_state'
#include "march_shared_msgs/msg/detail/low_voltage_state__traits.hpp"
// Member 'battery_state'
#include "march_shared_msgs/msg/detail/battery_state__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::msg::PowerDistributionBoardData>()
{
  return "march_shared_msgs::msg::PowerDistributionBoardData";
}

template<>
inline const char * name<march_shared_msgs::msg::PowerDistributionBoardData>()
{
  return "march_shared_msgs/msg/PowerDistributionBoardData";
}

template<>
struct has_fixed_size<march_shared_msgs::msg::PowerDistributionBoardData>
  : std::integral_constant<bool, has_fixed_size<march_shared_msgs::msg::BatteryState>::value && has_fixed_size<march_shared_msgs::msg::HighVoltageState>::value && has_fixed_size<march_shared_msgs::msg::LowVoltageState>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<march_shared_msgs::msg::PowerDistributionBoardData>
  : std::integral_constant<bool, has_bounded_size<march_shared_msgs::msg::BatteryState>::value && has_bounded_size<march_shared_msgs::msg::HighVoltageState>::value && has_bounded_size<march_shared_msgs::msg::LowVoltageState>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<march_shared_msgs::msg::PowerDistributionBoardData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__TRAITS_HPP_
