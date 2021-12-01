// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:srv/GetJointNames.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_JOINT_NAMES__TRAITS_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_JOINT_NAMES__TRAITS_HPP_

#include "march_shared_msgs/srv/detail/get_joint_names__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::GetJointNames_Request>()
{
  return "march_shared_msgs::srv::GetJointNames_Request";
}

template<>
inline const char * name<march_shared_msgs::srv::GetJointNames_Request>()
{
  return "march_shared_msgs/srv/GetJointNames_Request";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::GetJointNames_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::GetJointNames_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<march_shared_msgs::srv::GetJointNames_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::GetJointNames_Response>()
{
  return "march_shared_msgs::srv::GetJointNames_Response";
}

template<>
inline const char * name<march_shared_msgs::srv::GetJointNames_Response>()
{
  return "march_shared_msgs/srv/GetJointNames_Response";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::GetJointNames_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::GetJointNames_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<march_shared_msgs::srv::GetJointNames_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::GetJointNames>()
{
  return "march_shared_msgs::srv::GetJointNames";
}

template<>
inline const char * name<march_shared_msgs::srv::GetJointNames>()
{
  return "march_shared_msgs/srv/GetJointNames";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::GetJointNames>
  : std::integral_constant<
    bool,
    has_fixed_size<march_shared_msgs::srv::GetJointNames_Request>::value &&
    has_fixed_size<march_shared_msgs::srv::GetJointNames_Response>::value
  >
{
};

template<>
struct has_bounded_size<march_shared_msgs::srv::GetJointNames>
  : std::integral_constant<
    bool,
    has_bounded_size<march_shared_msgs::srv::GetJointNames_Request>::value &&
    has_bounded_size<march_shared_msgs::srv::GetJointNames_Response>::value
  >
{
};

template<>
struct is_service<march_shared_msgs::srv::GetJointNames>
  : std::true_type
{
};

template<>
struct is_service_request<march_shared_msgs::srv::GetJointNames_Request>
  : std::true_type
{
};

template<>
struct is_service_response<march_shared_msgs::srv::GetJointNames_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_JOINT_NAMES__TRAITS_HPP_
