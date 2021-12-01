// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryGoal.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__TRAITS_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__TRAITS_HPP_

#include "march_shared_msgs/msg/detail/follow_joint_trajectory_goal__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'trajectory'
#include "trajectory_msgs/msg/detail/joint_trajectory__traits.hpp"
// Member 'goal_time_tolerance'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<march_shared_msgs::msg::FollowJointTrajectoryGoal>()
{
  return "march_shared_msgs::msg::FollowJointTrajectoryGoal";
}

template<>
inline const char * name<march_shared_msgs::msg::FollowJointTrajectoryGoal>()
{
  return "march_shared_msgs/msg/FollowJointTrajectoryGoal";
}

template<>
struct has_fixed_size<march_shared_msgs::msg::FollowJointTrajectoryGoal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<march_shared_msgs::msg::FollowJointTrajectoryGoal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<march_shared_msgs::msg::FollowJointTrajectoryGoal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__TRAITS_HPP_
