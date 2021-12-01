// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionResult.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'status'
#include "actionlib_msgs/msg/detail/goal_status__struct.hpp"
// Member 'result'
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_result__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryActionResult __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryActionResult __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FollowJointTrajectoryActionResult_
{
  using Type = FollowJointTrajectoryActionResult_<ContainerAllocator>;

  explicit FollowJointTrajectoryActionResult_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    status(_init),
    result(_init)
  {
    (void)_init;
  }

  explicit FollowJointTrajectoryActionResult_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    status(_alloc, _init),
    result(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_type =
    actionlib_msgs::msg::GoalStatus_<ContainerAllocator>;
  _status_type status;
  using _result_type =
    march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status(
    const actionlib_msgs::msg::GoalStatus_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryActionResult
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryActionResult
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionResult_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowJointTrajectoryActionResult_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowJointTrajectoryActionResult_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowJointTrajectoryActionResult_

// alias to use template instance with default allocator
using FollowJointTrajectoryActionResult =
  march_shared_msgs::msg::FollowJointTrajectoryActionResult_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_RESULT__STRUCT_HPP_
