// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:srv/CapturePointPose.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__BUILDER_HPP_

#include "march_shared_msgs/srv/detail/capture_point_pose__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_CapturePointPose_Request_duration
{
public:
  Init_CapturePointPose_Request_duration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::march_shared_msgs::srv::CapturePointPose_Request duration(::march_shared_msgs::srv::CapturePointPose_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::CapturePointPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::CapturePointPose_Request>()
{
  return march_shared_msgs::srv::builder::Init_CapturePointPose_Request_duration();
}

}  // namespace march_shared_msgs


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_CapturePointPose_Response_capture_point
{
public:
  explicit Init_CapturePointPose_Response_capture_point(::march_shared_msgs::srv::CapturePointPose_Response & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::CapturePointPose_Response capture_point(::march_shared_msgs::srv::CapturePointPose_Response::_capture_point_type arg)
  {
    msg_.capture_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::CapturePointPose_Response msg_;
};

class Init_CapturePointPose_Response_duration
{
public:
  explicit Init_CapturePointPose_Response_duration(::march_shared_msgs::srv::CapturePointPose_Response & msg)
  : msg_(msg)
  {}
  Init_CapturePointPose_Response_capture_point duration(::march_shared_msgs::srv::CapturePointPose_Response::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_CapturePointPose_Response_capture_point(msg_);
  }

private:
  ::march_shared_msgs::srv::CapturePointPose_Response msg_;
};

class Init_CapturePointPose_Response_success
{
public:
  Init_CapturePointPose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CapturePointPose_Response_duration success(::march_shared_msgs::srv::CapturePointPose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CapturePointPose_Response_duration(msg_);
  }

private:
  ::march_shared_msgs::srv::CapturePointPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::CapturePointPose_Response>()
{
  return march_shared_msgs::srv::builder::Init_CapturePointPose_Response_success();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__BUILDER_HPP_
