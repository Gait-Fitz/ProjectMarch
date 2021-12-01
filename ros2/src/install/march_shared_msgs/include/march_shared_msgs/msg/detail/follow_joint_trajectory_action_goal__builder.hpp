// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionGoal.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_GOAL__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_GOAL__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_goal__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_FollowJointTrajectoryActionGoal_goal
{
public:
  explicit Init_FollowJointTrajectoryActionGoal_goal(::march_shared_msgs::msg::FollowJointTrajectoryActionGoal & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::FollowJointTrajectoryActionGoal goal(::march_shared_msgs::msg::FollowJointTrajectoryActionGoal::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryActionGoal msg_;
};

class Init_FollowJointTrajectoryActionGoal_goal_id
{
public:
  explicit Init_FollowJointTrajectoryActionGoal_goal_id(::march_shared_msgs::msg::FollowJointTrajectoryActionGoal & msg)
  : msg_(msg)
  {}
  Init_FollowJointTrajectoryActionGoal_goal goal_id(::march_shared_msgs::msg::FollowJointTrajectoryActionGoal::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FollowJointTrajectoryActionGoal_goal(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryActionGoal msg_;
};

class Init_FollowJointTrajectoryActionGoal_header
{
public:
  Init_FollowJointTrajectoryActionGoal_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowJointTrajectoryActionGoal_goal_id header(::march_shared_msgs::msg::FollowJointTrajectoryActionGoal::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FollowJointTrajectoryActionGoal_goal_id(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryActionGoal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::FollowJointTrajectoryActionGoal>()
{
  return march_shared_msgs::msg::builder::Init_FollowJointTrajectoryActionGoal_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_GOAL__BUILDER_HPP_
