// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryResult.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_RESULT__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_RESULT__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/follow_joint_trajectory_result__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_FollowJointTrajectoryResult_error_string
{
public:
  explicit Init_FollowJointTrajectoryResult_error_string(::march_shared_msgs::msg::FollowJointTrajectoryResult & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::FollowJointTrajectoryResult error_string(::march_shared_msgs::msg::FollowJointTrajectoryResult::_error_string_type arg)
  {
    msg_.error_string = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryResult msg_;
};

class Init_FollowJointTrajectoryResult_error_code
{
public:
  Init_FollowJointTrajectoryResult_error_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowJointTrajectoryResult_error_string error_code(::march_shared_msgs::msg::FollowJointTrajectoryResult::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_FollowJointTrajectoryResult_error_string(msg_);
  }

private:
  ::march_shared_msgs::msg::FollowJointTrajectoryResult msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::FollowJointTrajectoryResult>()
{
  return march_shared_msgs::msg::builder::Init_FollowJointTrajectoryResult_error_code();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_RESULT__BUILDER_HPP_
