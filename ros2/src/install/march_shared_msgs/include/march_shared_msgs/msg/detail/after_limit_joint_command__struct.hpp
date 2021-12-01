// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/AfterLimitJointCommand.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__AFTER_LIMIT_JOINT_COMMAND__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__AFTER_LIMIT_JOINT_COMMAND__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__AfterLimitJointCommand __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__AfterLimitJointCommand __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AfterLimitJointCommand_
{
  using Type = AfterLimitJointCommand_<ContainerAllocator>;

  explicit AfterLimitJointCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit AfterLimitJointCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _name_type name;
  using _position_command_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _position_command_type position_command;
  using _effort_command_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _effort_command_type effort_command;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__position_command(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->position_command = _arg;
    return *this;
  }
  Type & set__effort_command(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->effort_command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__AfterLimitJointCommand
    std::shared_ptr<march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__AfterLimitJointCommand
    std::shared_ptr<march_shared_msgs::msg::AfterLimitJointCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AfterLimitJointCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->position_command != other.position_command) {
      return false;
    }
    if (this->effort_command != other.effort_command) {
      return false;
    }
    return true;
  }
  bool operator!=(const AfterLimitJointCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AfterLimitJointCommand_

// alias to use template instance with default allocator
using AfterLimitJointCommand =
  march_shared_msgs::msg::AfterLimitJointCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__AFTER_LIMIT_JOINT_COMMAND__STRUCT_HPP_
