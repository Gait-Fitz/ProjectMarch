// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryResult.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_RESULT__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_RESULT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryResult __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryResult __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FollowJointTrajectoryResult_
{
  using Type = FollowJointTrajectoryResult_<ContainerAllocator>;

  explicit FollowJointTrajectoryResult_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_code = 0l;
      this->error_string = "";
    }
  }

  explicit FollowJointTrajectoryResult_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_string(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_code = 0l;
      this->error_string = "";
    }
  }

  // field types and members
  using _error_code_type =
    int32_t;
  _error_code_type error_code;
  using _error_string_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _error_string_type error_string;

  // setters for named parameter idiom
  Type & set__error_code(
    const int32_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__error_string(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->error_string = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t SUCCESSFUL =
    0;
  static constexpr int32_t INVALID_GOAL =
    -1;
  static constexpr int32_t INVALID_JOINTS =
    -2;
  static constexpr int32_t OLD_HEADER_TIMESTAMP =
    -3;
  static constexpr int32_t PATH_TOLERANCE_VIOLATED =
    -4;
  static constexpr int32_t GOAL_TOLERANCE_VIOLATED =
    -5;

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryResult
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryResult
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryResult_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowJointTrajectoryResult_ & other) const
  {
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->error_string != other.error_string) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowJointTrajectoryResult_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowJointTrajectoryResult_

// alias to use template instance with default allocator
using FollowJointTrajectoryResult =
  march_shared_msgs::msg::FollowJointTrajectoryResult_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int32_t FollowJointTrajectoryResult_<ContainerAllocator>::SUCCESSFUL;
template<typename ContainerAllocator>
constexpr int32_t FollowJointTrajectoryResult_<ContainerAllocator>::INVALID_GOAL;
template<typename ContainerAllocator>
constexpr int32_t FollowJointTrajectoryResult_<ContainerAllocator>::INVALID_JOINTS;
template<typename ContainerAllocator>
constexpr int32_t FollowJointTrajectoryResult_<ContainerAllocator>::OLD_HEADER_TIMESTAMP;
template<typename ContainerAllocator>
constexpr int32_t FollowJointTrajectoryResult_<ContainerAllocator>::PATH_TOLERANCE_VIOLATED;
template<typename ContainerAllocator>
constexpr int32_t FollowJointTrajectoryResult_<ContainerAllocator>::GOAL_TOLERANCE_VIOLATED;

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_RESULT__STRUCT_HPP_
