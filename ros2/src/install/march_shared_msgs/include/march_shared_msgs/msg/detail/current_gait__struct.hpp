// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/CurrentGait.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_GAIT__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_GAIT__STRUCT_HPP_

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
// Member 'duration'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__CurrentGait __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__CurrentGait __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CurrentGait_
{
  using Type = CurrentGait_<ContainerAllocator>;

  explicit CurrentGait_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    duration(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gait = "";
      this->subgait = "";
      this->version = "";
      this->gait_type = "";
    }
  }

  explicit CurrentGait_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    gait(_alloc),
    subgait(_alloc),
    version(_alloc),
    duration(_alloc, _init),
    gait_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gait = "";
      this->subgait = "";
      this->version = "";
      this->gait_type = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _gait_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _gait_type gait;
  using _subgait_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _subgait_type subgait;
  using _version_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _version_type version;
  using _duration_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _duration_type duration;
  using _gait_type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _gait_type_type gait_type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__gait(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->gait = _arg;
    return *this;
  }
  Type & set__subgait(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->subgait = _arg;
    return *this;
  }
  Type & set__version(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__duration(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__gait_type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->gait_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::CurrentGait_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::CurrentGait_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::CurrentGait_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::CurrentGait_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::CurrentGait_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::CurrentGait_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::CurrentGait_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::CurrentGait_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::CurrentGait_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::CurrentGait_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__CurrentGait
    std::shared_ptr<march_shared_msgs::msg::CurrentGait_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__CurrentGait
    std::shared_ptr<march_shared_msgs::msg::CurrentGait_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CurrentGait_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->gait != other.gait) {
      return false;
    }
    if (this->subgait != other.subgait) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    if (this->gait_type != other.gait_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const CurrentGait_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CurrentGait_

// alias to use template instance with default allocator
using CurrentGait =
  march_shared_msgs::msg::CurrentGait_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_GAIT__STRUCT_HPP_
