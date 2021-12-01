// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:srv/GetGaitParameters.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__BUILDER_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__BUILDER_HPP_

#include "march_shared_msgs/srv/detail/get_gait_parameters__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_GetGaitParameters_Request_subgait_name
{
public:
  explicit Init_GetGaitParameters_Request_subgait_name(::march_shared_msgs::srv::GetGaitParameters_Request & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::GetGaitParameters_Request subgait_name(::march_shared_msgs::srv::GetGaitParameters_Request::_subgait_name_type arg)
  {
    msg_.subgait_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::GetGaitParameters_Request msg_;
};

class Init_GetGaitParameters_Request_camera_to_use
{
public:
  explicit Init_GetGaitParameters_Request_camera_to_use(::march_shared_msgs::srv::GetGaitParameters_Request & msg)
  : msg_(msg)
  {}
  Init_GetGaitParameters_Request_subgait_name camera_to_use(::march_shared_msgs::srv::GetGaitParameters_Request::_camera_to_use_type arg)
  {
    msg_.camera_to_use = std::move(arg);
    return Init_GetGaitParameters_Request_subgait_name(msg_);
  }

private:
  ::march_shared_msgs::srv::GetGaitParameters_Request msg_;
};

class Init_GetGaitParameters_Request_realsense_category
{
public:
  Init_GetGaitParameters_Request_realsense_category()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetGaitParameters_Request_camera_to_use realsense_category(::march_shared_msgs::srv::GetGaitParameters_Request::_realsense_category_type arg)
  {
    msg_.realsense_category = std::move(arg);
    return Init_GetGaitParameters_Request_camera_to_use(msg_);
  }

private:
  ::march_shared_msgs::srv::GetGaitParameters_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::GetGaitParameters_Request>()
{
  return march_shared_msgs::srv::builder::Init_GetGaitParameters_Request_realsense_category();
}

}  // namespace march_shared_msgs


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_GetGaitParameters_Response_gait_parameters
{
public:
  explicit Init_GetGaitParameters_Response_gait_parameters(::march_shared_msgs::srv::GetGaitParameters_Response & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::GetGaitParameters_Response gait_parameters(::march_shared_msgs::srv::GetGaitParameters_Response::_gait_parameters_type arg)
  {
    msg_.gait_parameters = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::GetGaitParameters_Response msg_;
};

class Init_GetGaitParameters_Response_error_message
{
public:
  explicit Init_GetGaitParameters_Response_error_message(::march_shared_msgs::srv::GetGaitParameters_Response & msg)
  : msg_(msg)
  {}
  Init_GetGaitParameters_Response_gait_parameters error_message(::march_shared_msgs::srv::GetGaitParameters_Response::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_GetGaitParameters_Response_gait_parameters(msg_);
  }

private:
  ::march_shared_msgs::srv::GetGaitParameters_Response msg_;
};

class Init_GetGaitParameters_Response_success
{
public:
  Init_GetGaitParameters_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetGaitParameters_Response_error_message success(::march_shared_msgs::srv::GetGaitParameters_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetGaitParameters_Response_error_message(msg_);
  }

private:
  ::march_shared_msgs::srv::GetGaitParameters_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::GetGaitParameters_Response>()
{
  return march_shared_msgs::srv::builder::Init_GetGaitParameters_Response_success();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__BUILDER_HPP_
