// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:msg/PressureSolesData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__TRAITS_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__TRAITS_HPP_

#include "march_shared_msgs/msg/detail/pressure_soles_data__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'left_foot'
// Member 'right_foot'
#include "march_shared_msgs/msg/detail/pressure_sole_data__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::msg::PressureSolesData>()
{
  return "march_shared_msgs::msg::PressureSolesData";
}

template<>
inline const char * name<march_shared_msgs::msg::PressureSolesData>()
{
  return "march_shared_msgs/msg/PressureSolesData";
}

template<>
struct has_fixed_size<march_shared_msgs::msg::PressureSolesData>
  : std::integral_constant<bool, has_fixed_size<march_shared_msgs::msg::PressureSoleData>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<march_shared_msgs::msg::PressureSolesData>
  : std::integral_constant<bool, has_bounded_size<march_shared_msgs::msg::PressureSoleData>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<march_shared_msgs::msg::PressureSolesData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__TRAITS_HPP_
