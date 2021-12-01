// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:msg/PressureSoleData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__TRAITS_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__TRAITS_HPP_

#include "march_shared_msgs/msg/detail/pressure_sole_data__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::msg::PressureSoleData>()
{
  return "march_shared_msgs::msg::PressureSoleData";
}

template<>
inline const char * name<march_shared_msgs::msg::PressureSoleData>()
{
  return "march_shared_msgs/msg/PressureSoleData";
}

template<>
struct has_fixed_size<march_shared_msgs::msg::PressureSoleData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<march_shared_msgs::msg::PressureSoleData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<march_shared_msgs::msg::PressureSoleData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__TRAITS_HPP_
