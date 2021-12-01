// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:srv/GetMoveItTrajectory.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__TRAITS_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__TRAITS_HPP_

#include "march_shared_msgs/srv/detail/get_move_it_trajectory__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'swing_leg_target_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'stance_leg_target'
#include "sensor_msgs/msg/detail/joint_state__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::GetMoveItTrajectory_Request>()
{
  return "march_shared_msgs::srv::GetMoveItTrajectory_Request";
}

template<>
inline const char * name<march_shared_msgs::srv::GetMoveItTrajectory_Request>()
{
  return "march_shared_msgs/srv/GetMoveItTrajectory_Request";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::GetMoveItTrajectory_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::GetMoveItTrajectory_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<march_shared_msgs::srv::GetMoveItTrajectory_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'trajectory'
#include "trajectory_msgs/msg/detail/joint_trajectory__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::GetMoveItTrajectory_Response>()
{
  return "march_shared_msgs::srv::GetMoveItTrajectory_Response";
}

template<>
inline const char * name<march_shared_msgs::srv::GetMoveItTrajectory_Response>()
{
  return "march_shared_msgs/srv/GetMoveItTrajectory_Response";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::GetMoveItTrajectory_Response>
  : std::integral_constant<bool, has_fixed_size<trajectory_msgs::msg::JointTrajectory>::value> {};

template<>
struct has_bounded_size<march_shared_msgs::srv::GetMoveItTrajectory_Response>
  : std::integral_constant<bool, has_bounded_size<trajectory_msgs::msg::JointTrajectory>::value> {};

template<>
struct is_message<march_shared_msgs::srv::GetMoveItTrajectory_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::srv::GetMoveItTrajectory>()
{
  return "march_shared_msgs::srv::GetMoveItTrajectory";
}

template<>
inline const char * name<march_shared_msgs::srv::GetMoveItTrajectory>()
{
  return "march_shared_msgs/srv/GetMoveItTrajectory";
}

template<>
struct has_fixed_size<march_shared_msgs::srv::GetMoveItTrajectory>
  : std::integral_constant<
    bool,
    has_fixed_size<march_shared_msgs::srv::GetMoveItTrajectory_Request>::value &&
    has_fixed_size<march_shared_msgs::srv::GetMoveItTrajectory_Response>::value
  >
{
};

template<>
struct has_bounded_size<march_shared_msgs::srv::GetMoveItTrajectory>
  : std::integral_constant<
    bool,
    has_bounded_size<march_shared_msgs::srv::GetMoveItTrajectory_Request>::value &&
    has_bounded_size<march_shared_msgs::srv::GetMoveItTrajectory_Response>::value
  >
{
};

template<>
struct is_service<march_shared_msgs::srv::GetMoveItTrajectory>
  : std::true_type
{
};

template<>
struct is_service_request<march_shared_msgs::srv::GetMoveItTrajectory_Request>
  : std::true_type
{
};

template<>
struct is_service_response<march_shared_msgs::srv::GetMoveItTrajectory_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__TRAITS_HPP_
