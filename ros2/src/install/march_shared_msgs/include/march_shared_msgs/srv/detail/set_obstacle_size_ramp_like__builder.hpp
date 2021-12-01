// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:srv/SetObstacleSizeRampLike.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_RAMP_LIKE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_RAMP_LIKE__BUILDER_HPP_

#include "march_shared_msgs/srv/detail/set_obstacle_size_ramp_like__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_SetObstacleSizeRampLike_Request_new_ramp_up_length
{
public:
  explicit Init_SetObstacleSizeRampLike_Request_new_ramp_up_length(::march_shared_msgs::srv::SetObstacleSizeRampLike_Request & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::SetObstacleSizeRampLike_Request new_ramp_up_length(::march_shared_msgs::srv::SetObstacleSizeRampLike_Request::_new_ramp_up_length_type arg)
  {
    msg_.new_ramp_up_length = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeRampLike_Request msg_;
};

class Init_SetObstacleSizeRampLike_Request_new_slope_down
{
public:
  explicit Init_SetObstacleSizeRampLike_Request_new_slope_down(::march_shared_msgs::srv::SetObstacleSizeRampLike_Request & msg)
  : msg_(msg)
  {}
  Init_SetObstacleSizeRampLike_Request_new_ramp_up_length new_slope_down(::march_shared_msgs::srv::SetObstacleSizeRampLike_Request::_new_slope_down_type arg)
  {
    msg_.new_slope_down = std::move(arg);
    return Init_SetObstacleSizeRampLike_Request_new_ramp_up_length(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeRampLike_Request msg_;
};

class Init_SetObstacleSizeRampLike_Request_new_slope_up
{
public:
  explicit Init_SetObstacleSizeRampLike_Request_new_slope_up(::march_shared_msgs::srv::SetObstacleSizeRampLike_Request & msg)
  : msg_(msg)
  {}
  Init_SetObstacleSizeRampLike_Request_new_slope_down new_slope_up(::march_shared_msgs::srv::SetObstacleSizeRampLike_Request::_new_slope_up_type arg)
  {
    msg_.new_slope_up = std::move(arg);
    return Init_SetObstacleSizeRampLike_Request_new_slope_down(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeRampLike_Request msg_;
};

class Init_SetObstacleSizeRampLike_Request_obstacle_name
{
public:
  Init_SetObstacleSizeRampLike_Request_obstacle_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetObstacleSizeRampLike_Request_new_slope_up obstacle_name(::march_shared_msgs::srv::SetObstacleSizeRampLike_Request::_obstacle_name_type arg)
  {
    msg_.obstacle_name = std::move(arg);
    return Init_SetObstacleSizeRampLike_Request_new_slope_up(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeRampLike_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::SetObstacleSizeRampLike_Request>()
{
  return march_shared_msgs::srv::builder::Init_SetObstacleSizeRampLike_Request_obstacle_name();
}

}  // namespace march_shared_msgs


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_SetObstacleSizeRampLike_Response_success
{
public:
  Init_SetObstacleSizeRampLike_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::march_shared_msgs::srv::SetObstacleSizeRampLike_Response success(::march_shared_msgs::srv::SetObstacleSizeRampLike_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeRampLike_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::SetObstacleSizeRampLike_Response>()
{
  return march_shared_msgs::srv::builder::Init_SetObstacleSizeRampLike_Response_success();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_RAMP_LIKE__BUILDER_HPP_
