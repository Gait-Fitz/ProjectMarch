// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:srv/PublishTestDataset.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__PUBLISH_TEST_DATASET__BUILDER_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__PUBLISH_TEST_DATASET__BUILDER_HPP_

#include "march_shared_msgs/srv/detail/publish_test_dataset__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_PublishTestDataset_Request_save_camera_back
{
public:
  explicit Init_PublishTestDataset_Request_save_camera_back(::march_shared_msgs::srv::PublishTestDataset_Request & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::PublishTestDataset_Request save_camera_back(::march_shared_msgs::srv::PublishTestDataset_Request::_save_camera_back_type arg)
  {
    msg_.save_camera_back = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::PublishTestDataset_Request msg_;
};

class Init_PublishTestDataset_Request_pointcloud_file_name
{
public:
  explicit Init_PublishTestDataset_Request_pointcloud_file_name(::march_shared_msgs::srv::PublishTestDataset_Request & msg)
  : msg_(msg)
  {}
  Init_PublishTestDataset_Request_save_camera_back pointcloud_file_name(::march_shared_msgs::srv::PublishTestDataset_Request::_pointcloud_file_name_type arg)
  {
    msg_.pointcloud_file_name = std::move(arg);
    return Init_PublishTestDataset_Request_save_camera_back(msg_);
  }

private:
  ::march_shared_msgs::srv::PublishTestDataset_Request msg_;
};

class Init_PublishTestDataset_Request_selected_mode
{
public:
  Init_PublishTestDataset_Request_selected_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PublishTestDataset_Request_pointcloud_file_name selected_mode(::march_shared_msgs::srv::PublishTestDataset_Request::_selected_mode_type arg)
  {
    msg_.selected_mode = std::move(arg);
    return Init_PublishTestDataset_Request_pointcloud_file_name(msg_);
  }

private:
  ::march_shared_msgs::srv::PublishTestDataset_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::PublishTestDataset_Request>()
{
  return march_shared_msgs::srv::builder::Init_PublishTestDataset_Request_selected_mode();
}

}  // namespace march_shared_msgs


namespace march_shared_msgs
{

namespace srv
{

namespace builder
{

class Init_PublishTestDataset_Response_message
{
public:
  explicit Init_PublishTestDataset_Response_message(::march_shared_msgs::srv::PublishTestDataset_Response & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::srv::PublishTestDataset_Response message(::march_shared_msgs::srv::PublishTestDataset_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::srv::PublishTestDataset_Response msg_;
};

class Init_PublishTestDataset_Response_success
{
public:
  Init_PublishTestDataset_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PublishTestDataset_Response_message success(::march_shared_msgs::srv::PublishTestDataset_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PublishTestDataset_Response_message(msg_);
  }

private:
  ::march_shared_msgs::srv::PublishTestDataset_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::srv::PublishTestDataset_Response>()
{
  return march_shared_msgs::srv::builder::Init_PublishTestDataset_Response_success();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__PUBLISH_TEST_DATASET__BUILDER_HPP_
