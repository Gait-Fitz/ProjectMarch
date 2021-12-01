// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:srv/SetObstacleSizeBlockLike.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_BLOCK_LIKE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_BLOCK_LIKE__BUILDER_HPP_

#include "march_shared_msgs/srv/detail/set_obstacle_size_block_like__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_SetObstacleSizeBlockLike_Request_new_length
{
public:
  explicit Init_SetObstacleSizeBlockLike_Request_new_length(::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request new_length(::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request::_new_length_type arg)
  {
    msg_.new_length = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request msg_;
};

class Init_SetObstacleSizeBlockLike_Request_new_width
{
public:
  explicit Init_SetObstacleSizeBlockLike_Request_new_width(::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request & msg)
  : msg_(msg)
  {}
  Init_SetObstacleSizeBlockLike_Request_new_length new_width(::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request::_new_width_type arg)
  {
    msg_.new_width = std::move(arg);
    return Init_SetObstacleSizeBlockLike_Request_new_length(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request msg_;
};

class Init_SetObstacleSizeBlockLike_Request_new_height
{
public:
  explicit Init_SetObstacleSizeBlockLike_Request_new_height(::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request & msg)
  : msg_(msg)
  {}
  Init_SetObstacleSizeBlockLike_Request_new_width new_height(::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request::_new_height_type arg)
  {
    msg_.new_height = std::move(arg);
    return Init_SetObstacleSizeBlockLike_Request_new_width(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request msg_;
};

class Init_SetObstacleSizeBlockLike_Request_obstacle_name
{
public:
  Init_SetObstacleSizeBlockLike_Request_obstacle_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetObstacleSizeBlockLike_Request_new_height obstacle_name(::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request::_obstacle_name_type arg)
  {
    msg_.obstacle_name = std::move(arg);
    return Init_SetObstacleSizeBlockLike_Request_new_height(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::SetObstacleSizeBlockLike_Request>()
{
  return march_shared_msgs::srv::builder::Init_SetObstacleSizeBlockLike_Request_obstacle_name();
}

}  // namespace march_shared_msgs


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_SetObstacleSizeBlockLike_Response_success
{
public:
  Init_SetObstacleSizeBlockLike_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::march_shared_msgs::srv::SetObstacleSizeBlockLike_Response success(::march_shared_msgs::srv::SetObstacleSizeBlockLike_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::SetObstacleSizeBlockLike_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::SetObstacleSizeBlockLike_Response>()
{
  return march_shared_msgs::srv::builder::Init_SetObstacleSizeBlockLike_Response_success();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_BLOCK_LIKE__BUILDER_HPP_
