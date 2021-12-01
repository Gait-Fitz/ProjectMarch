// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:srv/CapturePointPose.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__TRAITS_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__TRAITS_HPP_

#include "march_shared_msgs/srv/detail/capture_point_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::CapturePointPose_Request>()
{
  return "march_shared_msgs::srv::CapturePointPose_Request";
}

template<>
inline const char * name<march_shared_msgs::srv::CapturePointPose_Request>()
{
  return "march_shared_msgs/srv/CapturePointPose_Request";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::CapturePointPose_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::CapturePointPose_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<march_shared_msgs::srv::CapturePointPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'capture_point'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::CapturePointPose_Response>()
{
  return "march_shared_msgs::srv::CapturePointPose_Response";
}

template<>
inline const char * name<march_shared_msgs::srv::CapturePointPose_Response>()
{
  return "march_shared_msgs/srv/CapturePointPose_Response";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::CapturePointPose_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::CapturePointPose_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<march_shared_msgs::srv::CapturePointPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::CapturePointPose>()
{
  return "march_shared_msgs::srv::CapturePointPose";
}

template<>
inline const char * name<march_shared_msgs::srv::CapturePointPose>()
{
  return "march_shared_msgs/srv/CapturePointPose";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::CapturePointPose>
  : std::integral_constant<
    bool,
    has_fixed_size<march_shared_msgs::srv::CapturePointPose_Request>::value &&
    has_fixed_size<march_shared_msgs::srv::CapturePointPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<march_shared_msgs::srv::CapturePointPose>
  : std::integral_constant<
    bool,
    has_bounded_size<march_shared_msgs::srv::CapturePointPose_Request>::value &&
    has_bounded_size<march_shared_msgs::srv::CapturePointPose_Response>::value
  >
{
};

template<>
struct is_service<march_shared_msgs::srv::CapturePointPose>
  : std::true_type
{
};

template<>
struct is_service_request<march_shared_msgs::srv::CapturePointPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<march_shared_msgs::srv::CapturePointPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__TRAITS_HPP_
