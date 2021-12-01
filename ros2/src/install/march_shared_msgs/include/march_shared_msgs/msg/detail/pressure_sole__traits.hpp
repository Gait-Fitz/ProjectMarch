// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:msg/PressureSole.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__TRAITS_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__TRAITS_HPP_

#include "march_shared_msgs/msg/detail/pressure_sole__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pressure_soles_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::msg::PressureSole>()
{
  return "march_shared_msgs::msg::PressureSole";
}

template<>
inline const char * name<march_shared_msgs::msg::PressureSole>()
{
  return "march_shared_msgs/msg/PressureSole";
}

template<>
struct has_fixed_size<march_shared_msgs::msg::PressureSole>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<march_shared_msgs::msg::PressureSole>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<march_shared_msgs::msg::PressureSole>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__TRAITS_HPP_
