// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionResult.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/follow_joint_trajectory_action_result__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_FollowJointTrajectoryActionResult_result
{
public:
  explicit Init_FollowJointTrajectoryActionResult_result(::march_shared_msgs::msg::FollowJointTrajectoryActionResult & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::FollowJointTrajectoryActionResult result(::march_shared_msgs::msg::FollowJointTrajectoryActionResult::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryActionResult msg_;
};

class Init_FollowJointTrajectoryActionResult_status
{
public:
  explicit Init_FollowJointTrajectoryActionResult_status(::march_shared_msgs::msg::FollowJointTrajectoryActionResult & msg)
  : msg_(msg)
  {}
  Init_FollowJointTrajectoryActionResult_result status(::march_shared_msgs::msg::FollowJointTrajectoryActionResult::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FollowJointTrajectoryActionResult_result(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryActionResult msg_;
};

class Init_FollowJointTrajectoryActionResult_header
{
public:
  Init_FollowJointTrajectoryActionResult_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowJointTrajectoryActionResult_status header(::march_shared_msgs::msg::FollowJointTrajectoryActionResult::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FollowJointTrajectoryActionResult_status(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryActionResult msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::FollowJointTrajectoryActionResult>()
{
  return march_shared_msgs::msg::builder::Init_FollowJointTrajectoryActionResult_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__BUILDER_HPP_
