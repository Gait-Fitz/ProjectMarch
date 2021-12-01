// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:srv/SetGaitVersion.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__SET_GAIT_VERSION__BUILDER_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__SET_GAIT_VERSION__BUILDER_HPP_

#include "march_shared_msgs/srv/detail/set_gait_version__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_SetGaitVersion_Request_versions
{
public:
  explicit Init_SetGaitVersion_Request_versions(::march_shared_msgs::srv::SetGaitVersion_Request & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::SetGaitVersion_Request versions(::march_shared_msgs::srv::SetGaitVersion_Request::_versions_type arg)
  {
    msg_.versions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::SetGaitVersion_Request msg_;
};

class Init_SetGaitVersion_Request_subgaits
{
public:
  explicit Init_SetGaitVersion_Request_subgaits(::march_shared_msgs::srv::SetGaitVersion_Request & msg)
  : msg_(msg)
  {}
  Init_SetGaitVersion_Request_versions subgaits(::march_shared_msgs::srv::SetGaitVersion_Request::_subgaits_type arg)
  {
    msg_.subgaits = std::move(arg);
    return Init_SetGaitVersion_Request_versions(msg_);
  }

private:
  ::march_shared_msgs::srv::SetGaitVersion_Request msg_;
};

class Init_SetGaitVersion_Request_gait
{
public:
  Init_SetGaitVersion_Request_gait()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetGaitVersion_Request_subgaits gait(::march_shared_msgs::srv::SetGaitVersion_Request::_gait_type arg)
  {
    msg_.gait = std::move(arg);
    return Init_SetGaitVersion_Request_subgaits(msg_);
  }

private:
  ::march_shared_msgs::srv::SetGaitVersion_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::SetGaitVersion_Request>()
{
  return march_shared_msgs::srv::builder::Init_SetGaitVersion_Request_gait();
}

}  // namespace march_shared_msgs


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_SetGaitVersion_Response_message
{
public:
  explicit Init_SetGaitVersion_Response_message(::march_shared_msgs::srv::SetGaitVersion_Response & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::SetGaitVersion_Response message(::march_shared_msgs::srv::SetGaitVersion_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::SetGaitVersion_Response msg_;
};

class Init_SetGaitVersion_Response_success
{
public:
  Init_SetGaitVersion_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetGaitVersion_Response_message success(::march_shared_msgs::srv::SetGaitVersion_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetGaitVersion_Response_message(msg_);
  }

private:
  ::march_shared_msgs::srv::SetGaitVersion_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::SetGaitVersion_Response>()
{
  return march_shared_msgs::srv::builder::Init_SetGaitVersion_Response_success();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__SET_GAIT_VERSION__BUILDER_HPP_
