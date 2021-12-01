// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:msg/MpcJoint.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__TRAITS_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__TRAITS_HPP_

#include "march_shared_msgs/msg/detail/mpc_joint__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'estimation'
// Member 'reference'
#include "march_shared_msgs/msg/detail/mpc_state_vectors__traits.hpp"
// Member 'tuning'
#include "march_shared_msgs/msg/detail/mpc_tuning__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::msg::MpcJoint>()
{
  return "march_shared_msgs::msg::MpcJoint";
}

template<>
inline const char * name<march_shared_msgs::msg::MpcJoint>()
{
  return "march_shared_msgs/msg/MpcJoint";
}

template<>
struct has_fixed_size<march_shared_msgs::msg::MpcJoint>
  : std::integral_constant<bool, has_fixed_size<march_shared_msgs::msg::MpcStateVectors>::value && has_fixed_size<march_shared_msgs::msg::MpcTuning>::value> {};

template<>
struct has_bounded_size<march_shared_msgs::msg::MpcJoint>
  : std::integral_constant<bool, has_bounded_size<march_shared_msgs::msg::MpcStateVectors>::value && has_bounded_size<march_shared_msgs::msg::MpcTuning>::value> {};

template<>
struct is_message<march_shared_msgs::msg::MpcJoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__TRAITS_HPP_
