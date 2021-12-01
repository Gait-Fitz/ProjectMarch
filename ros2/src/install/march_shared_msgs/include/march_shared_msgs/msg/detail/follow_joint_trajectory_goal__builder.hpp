// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryGoal.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/follow_joint_trajectory_goal__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_FollowJointTrajectoryGoal_goal_time_tolerance
{
public:
  explicit Init_FollowJointTrajectoryGoal_goal_time_tolerance(::march_shared_msgs::msg::FollowJointTrajectoryGoal & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::FollowJointTrajectoryGoal goal_time_tolerance(::march_shared_msgs::msg::FollowJointTrajectoryGoal::_goal_time_tolerance_type arg)
  {
    msg_.goal_time_tolerance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryGoal msg_;
};

class Init_FollowJointTrajectoryGoal_goal_tolerance
{
public:
  explicit Init_FollowJointTrajectoryGoal_goal_tolerance(::march_shared_msgs::msg::FollowJointTrajectoryGoal & msg)
  : msg_(msg)
  {}
  Init_FollowJointTrajectoryGoal_goal_time_tolerance goal_tolerance(::march_shared_msgs::msg::FollowJointTrajectoryGoal::_goal_tolerance_type arg)
  {
    msg_.goal_tolerance = std::move(arg);
    return Init_FollowJointTrajectoryGoal_goal_time_tolerance(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryGoal msg_;
};

class Init_FollowJointTrajectoryGoal_path_tolerance
{
public:
  explicit Init_FollowJointTrajectoryGoal_path_tolerance(::march_shared_msgs::msg::FollowJointTrajectoryGoal & msg)
  : msg_(msg)
  {}
  Init_FollowJointTrajectoryGoal_goal_tolerance path_tolerance(::march_shared_msgs::msg::FollowJointTrajectoryGoal::_path_tolerance_type arg)
  {
    msg_.path_tolerance = std::move(arg);
    return Init_FollowJointTrajectoryGoal_goal_tolerance(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryGoal msg_;
};

class Init_FollowJointTrajectoryGoal_trajectory
{
public:
  Init_FollowJointTrajectoryGoal_trajectory()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowJointTrajectoryGoal_path_tolerance trajectory(::march_shared_msgs::msg::FollowJointTrajectoryGoal::_trajectory_type arg)
  {
    msg_.trajectory = std::move(arg);
    return Init_FollowJointTrajectoryGoal_path_tolerance(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryGoal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::FollowJointTrajectoryGoal>()
{
  return march_shared_msgs::msg::builder::Init_FollowJointTrajectoryGoal_trajectory();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__BUILDER_HPP_
