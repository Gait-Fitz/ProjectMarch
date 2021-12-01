// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:srv/PossibleGaits.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__POSSIBLE_GAITS__TRAITS_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__POSSIBLE_GAITS__TRAITS_HPP_

#include "march_shared_msgs/srv/detail/possible_gaits__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::PossibleGaits_Request>()
{
  return "march_shared_msgs::srv::PossibleGaits_Request";
}

template<>
inline const char * name<march_shared_msgs::srv::PossibleGaits_Request>()
{
  return "march_shared_msgs/srv/PossibleGaits_Request";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::PossibleGaits_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::PossibleGaits_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<march_shared_msgs::srv::PossibleGaits_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::PossibleGaits_Response>()
{
  return "march_shared_msgs::srv::PossibleGaits_Response";
}

template<>
inline const char * name<march_shared_msgs::srv::PossibleGaits_Response>()
{
  return "march_shared_msgs/srv/PossibleGaits_Response";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::PossibleGaits_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::PossibleGaits_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<march_shared_msgs::srv::PossibleGaits_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::PossibleGaits>()
{
  return "march_shared_msgs::srv::PossibleGaits";
}

template<>
inline const char * name<march_shared_msgs::srv::PossibleGaits>()
{
  return "march_shared_msgs/srv/PossibleGaits";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::PossibleGaits>
  : std::integral_constant<
    bool,
    has_fixed_size<march_shared_msgs::srv::PossibleGaits_Request>::value &&
    has_fixed_size<march_shared_msgs::srv::PossibleGaits_Response>::value
  >
{
};

template<>
struct has_bounded_size<march_shared_msgs::srv::PossibleGaits>
  : std::integral_constant<
    bool,
    has_bounded_size<march_shared_msgs::srv::PossibleGaits_Request>::value &&
    has_bounded_size<march_shared_msgs::srv::PossibleGaits_Response>::value
  >
{
};

template<>
struct is_service<march_shared_msgs::srv::PossibleGaits>
  : std::true_type
{
};

template<>
struct is_service_request<march_shared_msgs::srv::PossibleGaits_Request>
  : std::true_type
{
};

template<>
struct is_service_response<march_shared_msgs::srv::PossibleGaits_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__POSSIBLE_GAITS__TRAITS_HPP_
