// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionFeedback.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_FEEDBACK__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_FEEDBACK__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_feedback__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_FollowJointTrajectoryActionFeedback_feedback
{
public:
  explicit Init_FollowJointTrajectoryActionFeedback_feedback(::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback feedback(::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback msg_;
};

class Init_FollowJointTrajectoryActionFeedback_status
{
public:
  explicit Init_FollowJointTrajectoryActionFeedback_status(::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback & msg)
  : msg_(msg)
  {}
  Init_FollowJointTrajectoryActionFeedback_feedback status(::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FollowJointTrajectoryActionFeedback_feedback(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback msg_;
};

class Init_FollowJointTrajectoryActionFeedback_header
{
public:
  Init_FollowJointTrajectoryActionFeedback_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowJointTrajectoryActionFeedback_status header(::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FollowJointTrajectoryActionFeedback_status(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::FollowJointTrajectoryActionFeedback>()
{
  return march_shared_msgs::msg::builder::Init_FollowJointTrajectoryActionFeedback_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_FEEDBACK__BUILDER_HPP_
