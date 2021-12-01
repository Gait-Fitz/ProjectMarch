// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionResult.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__TRAITS_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__TRAITS_HPP_

#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_result__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'status'
#include "actionlib_msgs/msg/detail/goal_status__traits.hpp"
// Member 'result'
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_result__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::msg::FollowJointTrajectoryActionResult>()
{
  return "march_shared_msgs::msg::FollowJointTrajectoryActionResult";
}

template<>
inline const char * name<march_shared_msgs::msg::FollowJointTrajectoryActionResult>()
{
  return "march_shared_msgs/msg/FollowJointTrajectoryActionResult";
}

template<>
struct has_fixed_size<march_shared_msgs::msg::FollowJointTrajectoryActionResult>
  : std::integral_constant<bool, has_fixed_size<actionlib_msgs::msg::GoalStatus>::value && has_fixed_size<march_shared_msgs::msg::FollowJointTrajectoryResult>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<march_shared_msgs::msg::FollowJointTrajectoryActionResult>
  : std::integral_constant<bool, has_bounded_size<actionlib_msgs::msg::GoalStatus>::value && has_bounded_size<march_shared_msgs::msg::FollowJointTrajectoryResult>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<march_shared_msgs::msg::FollowJointTrajectoryActionResult>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__TRAITS_HPP_
