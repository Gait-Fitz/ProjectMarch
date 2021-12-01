// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryFeedback.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_FEEDBACK__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_FEEDBACK__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/follow_joint_trajectory_feedback__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_FollowJointTrajectoryFeedback_error
{
public:
  explicit Init_FollowJointTrajectoryFeedback_error(::march_shared_msgs::msg::FollowJointTrajectoryFeedback & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::FollowJointTrajectoryFeedback error(::march_shared_msgs::msg::FollowJointTrajectoryFeedback::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryFeedback msg_;
};

class Init_FollowJointTrajectoryFeedback_actual
{
public:
  explicit Init_FollowJointTrajectoryFeedback_actual(::march_shared_msgs::msg::FollowJointTrajectoryFeedback & msg)
  : msg_(msg)
  {}
  Init_FollowJointTrajectoryFeedback_error actual(::march_shared_msgs::msg::FollowJointTrajectoryFeedback::_actual_type arg)
  {
    msg_.actual = std::move(arg);
    return Init_FollowJointTrajectoryFeedback_error(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryFeedback msg_;
};

class Init_FollowJointTrajectoryFeedback_desired
{
public:
  explicit Init_FollowJointTrajectoryFeedback_desired(::march_shared_msgs::msg::FollowJointTrajectoryFeedback & msg)
  : msg_(msg)
  {}
  Init_FollowJointTrajectoryFeedback_actual desired(::march_shared_msgs::msg::FollowJointTrajectoryFeedback::_desired_type arg)
  {
    msg_.desired = std::move(arg);
    return Init_FollowJointTrajectoryFeedback_actual(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryFeedback msg_;
};

class Init_FollowJointTrajectoryFeedback_joint_names
{
public:
  explicit Init_FollowJointTrajectoryFeedback_joint_names(::march_shared_msgs::msg::FollowJointTrajectoryFeedback & msg)
  : msg_(msg)
  {}
  Init_FollowJointTrajectoryFeedback_desired joint_names(::march_shared_msgs::msg::FollowJointTrajectoryFeedback::_joint_names_type arg)
  {
    msg_.joint_names = std::move(arg);
    return Init_FollowJointTrajectoryFeedback_desired(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryFeedback msg_;
};

class Init_FollowJointTrajectoryFeedback_header
{
public:
  Init_FollowJointTrajectoryFeedback_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowJointTrajectoryFeedback_joint_names header(::march_shared_msgs::msg::FollowJointTrajectoryFeedback::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FollowJointTrajectoryFeedback_joint_names(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::FollowJointTrajectoryFeedback>()
{
  return march_shared_msgs::msg::builder::Init_FollowJointTrajectoryFeedback_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_FEEDBACK__BUILDER_HPP_
