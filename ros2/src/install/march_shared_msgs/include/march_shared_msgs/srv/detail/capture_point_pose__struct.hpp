// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:srv/CapturePointPose.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__CapturePointPose_Request __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__CapturePointPose_Request __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CapturePointPose_Request_
{
  using Type = CapturePointPose_Request_<ContainerAllocator>;

  explicit CapturePointPose_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->duration = 0.0;
    }
  }

  explicit CapturePointPose_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->duration = 0.0;
    }
  }

  // field types and members
  using _duration_type =
    double;
  _duration_type duration;

  // setters for named parameter idiom
  Type & set__duration(
    const double & _arg)
  {
    this->duration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__CapturePointPose_Request
    std::shared_ptr<march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__CapturePointPose_Request
    std::shared_ptr<march_shared_msgs::srv::CapturePointPose_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CapturePointPose_Request_ & other) const
  {
    if (this->duration != other.duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const CapturePointPose_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CapturePointPose_Request_

// alias to use template instance with default allocator
using CapturePointPose_Request =
  march_shared_msgs::srv::CapturePointPose_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs


// Include directives for member types
// Member 'capture_point'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__CapturePointPose_Response __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__CapturePointPose_Response __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CapturePointPose_Response_
{
  using Type = CapturePointPose_Response_<ContainerAllocator>;

  explicit CapturePointPose_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : capture_point(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->duration = 0.0;
    }
  }

  explicit CapturePointPose_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : capture_point(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->duration = 0.0;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _duration_type =
    double;
  _duration_type duration;
  using _capture_point_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _capture_point_type capture_point;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__duration(
    const double & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__capture_point(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->capture_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__CapturePointPose_Response
    std::shared_ptr<march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__CapturePointPose_Response
    std::shared_ptr<march_shared_msgs::srv::CapturePointPose_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CapturePointPose_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    if (this->capture_point != other.capture_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const CapturePointPose_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CapturePointPose_Response_

// alias to use template instance with default allocator
using CapturePointPose_Response =
  march_shared_msgs::srv::CapturePointPose_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs

namespace march_shared_msgs
{

namespace srv
{

struct CapturePointPose
{
  using Request = march_shared_msgs::srv::CapturePointPose_Request;
  using Response = march_shared_msgs::srv::CapturePointPose_Response;
};

}  // namespace srv

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__CAPTURE_POINT_POSE__STRUCT_HPP_
