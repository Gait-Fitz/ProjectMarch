// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryFeedback.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_FEEDBACK__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_FEEDBACK__STRUCT_HPP_

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
// Member 'desired'
// Member 'actual'
// Member 'error'
#include "trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryFeedback __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryFeedback __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FollowJointTrajectoryFeedback_
{
  using Type = FollowJointTrajectoryFeedback_<ContainerAllocator>;

  explicit FollowJointTrajectoryFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    desired(_init),
    actual(_init),
    error(_init)
  {
    (void)_init;
  }

  explicit FollowJointTrajectoryFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    desired(_alloc, _init),
    actual(_alloc, _init),
    error(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _joint_names_type joint_names;
  using _desired_type =
    trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>;
  _desired_type desired;
  using _actual_type =
    trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>;
  _actual_type actual;
  using _error_type =
    trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>;
  _error_type error;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->joint_names = _arg;
    return *this;
  }
  Type & set__desired(
    const trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator> & _arg)
  {
    this->desired = _arg;
    return *this;
  }
  Type & set__actual(
    const trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator> & _arg)
  {
    this->actual = _arg;
    return *this;
  }
  Type & set__error(
    const trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator> & _arg)
  {
    this->error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryFeedback
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryFeedback
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowJointTrajectoryFeedback_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_names != other.joint_names) {
      return false;
    }
    if (this->desired != other.desired) {
      return false;
    }
    if (this->actual != other.actual) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowJointTrajectoryFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowJointTrajectoryFeedback_

// alias to use template instance with default allocator
using FollowJointTrajectoryFeedback =
  march_shared_msgs::msg::FollowJointTrajectoryFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_FEEDBACK__STRUCT_HPP_
