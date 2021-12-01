// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:srv/GetGaitParameters.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__TRAITS_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__TRAITS_HPP_

#include "march_shared_msgs/srv/detail/get_gait_parameters__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::GetGaitParameters_Request>()
{
  return "march_shared_msgs::srv::GetGaitParameters_Request";
}

template<>
inline const char * name<march_shared_msgs::srv::GetGaitParameters_Request>()
{
  return "march_shared_msgs/srv/GetGaitParameters_Request";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::GetGaitParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::GetGaitParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<march_shared_msgs::srv::GetGaitParameters_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'gait_parameters'
#include "march_shared_msgs/msg/detail/gait_parameters__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::GetGaitParameters_Response>()
{
  return "march_shared_msgs::srv::GetGaitParameters_Response";
}

template<>
inline const char * name<march_shared_msgs::srv::GetGaitParameters_Response>()
{
  return "march_shared_msgs/srv/GetGaitParameters_Response";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::GetGaitParameters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::GetGaitParameters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<march_shared_msgs::srv::GetGaitParameters_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::GetGaitParameters>()
{
  return "march_shared_msgs::srv::GetGaitParameters";
}

template<>
inline const char * name<march_shared_msgs::srv::GetGaitParameters>()
{
  return "march_shared_msgs/srv/GetGaitParameters";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::GetGaitParameters>
  : std::integral_constant<
    bool,
    has_fixed_size<march_shared_msgs::srv::GetGaitParameters_Request>::value &&
    has_fixed_size<march_shared_msgs::srv::GetGaitParameters_Response>::value
  >
{
};

template<>
struct has_bounded_size<march_shared_msgs::srv::GetGaitParameters>
  : std::integral_constant<
    bool,
    has_bounded_size<march_shared_msgs::srv::GetGaitParameters_Request>::value &&
    has_bounded_size<march_shared_msgs::srv::GetGaitParameters_Response>::value
  >
{
};

template<>
struct is_service<march_shared_msgs::srv::GetGaitParameters>
  : std::true_type
{
};

template<>
struct is_service_request<march_shared_msgs::srv::GetGaitParameters_Request>
  : std::true_type
{
};

template<>
struct is_service_response<march_shared_msgs::srv::GetGaitParameters_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__TRAITS_HPP_
