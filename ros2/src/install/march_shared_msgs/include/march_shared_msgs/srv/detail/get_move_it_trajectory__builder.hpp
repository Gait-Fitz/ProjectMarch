// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:srv/GetMoveItTrajectory.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__BUILDER_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__BUILDER_HPP_

#include "march_shared_msgs/srv/detail/get_move_it_trajectory__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_GetMoveItTrajectory_Request_stance_leg_target
{
public:
  explicit Init_GetMoveItTrajectory_Request_stance_leg_target(::march_shared_msgs::srv::GetMoveItTrajectory_Request & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::GetMoveItTrajectory_Request stance_leg_target(::march_shared_msgs::srv::GetMoveItTrajectory_Request::_stance_leg_target_type arg)
  {
    msg_.stance_leg_target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::GetMoveItTrajectory_Request msg_;
};

class Init_GetMoveItTrajectory_Request_swing_leg_target_pose
{
public:
  explicit Init_GetMoveItTrajectory_Request_swing_leg_target_pose(::march_shared_msgs::srv::GetMoveItTrajectory_Request & msg)
  : msg_(msg)
  {}
  Init_GetMoveItTrajectory_Request_stance_leg_target swing_leg_target_pose(::march_shared_msgs::srv::GetMoveItTrajectory_Request::_swing_leg_target_pose_type arg)
  {
    msg_.swing_leg_target_pose = std::move(arg);
    return Init_GetMoveItTrajectory_Request_stance_leg_target(msg_);
  }

private:
  ::march_shared_msgs::srv::GetMoveItTrajectory_Request msg_;
};

class Init_GetMoveItTrajectory_Request_swing_leg
{
public:
  Init_GetMoveItTrajectory_Request_swing_leg()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetMoveItTrajectory_Request_swing_leg_target_pose swing_leg(::march_shared_msgs::srv::GetMoveItTrajectory_Request::_swing_leg_type arg)
  {
    msg_.swing_leg = std::move(arg);
    return Init_GetMoveItTrajectory_Request_swing_leg_target_pose(msg_);
  }

private:
  ::march_shared_msgs::srv::GetMoveItTrajectory_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::GetMoveItTrajectory_Request>()
{
  return march_shared_msgs::srv::builder::Init_GetMoveItTrajectory_Request_swing_leg();
}

}  // namespace march_shared_msgs


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_GetMoveItTrajectory_Response_trajectory
{
public:
  explicit Init_GetMoveItTrajectory_Response_trajectory(::march_shared_msgs::srv::GetMoveItTrajectory_Response & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::GetMoveItTrajectory_Response trajectory(::march_shared_msgs::srv::GetMoveItTrajectory_Response::_trajectory_type arg)
  {
    msg_.trajectory = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::GetMoveItTrajectory_Response msg_;
};

class Init_GetMoveItTrajectory_Response_success
{
public:
  Init_GetMoveItTrajectory_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetMoveItTrajectory_Response_trajectory success(::march_shared_msgs::srv::GetMoveItTrajectory_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetMoveItTrajectory_Response_trajectory(msg_);
  }

private:
  ::march_shared_msgs::srv::GetMoveItTrajectory_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::GetMoveItTrajectory_Response>()
{
  return march_shared_msgs::srv::builder::Init_GetMoveItTrajectory_Response_success();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__BUILDER_HPP_
