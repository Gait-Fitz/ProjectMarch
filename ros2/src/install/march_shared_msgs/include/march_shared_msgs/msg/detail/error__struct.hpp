// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/Error.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__ERROR__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__ERROR__STRUCT_HPP_

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
# define DEPRECATED__march_shared_msgs__msg__Error __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__Error __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Error_
{
  using Type = Error_<ContainerAllocator>;

  explicit Error_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_message = "";
      this->type = 0;
    }
  }

  explicit Error_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    error_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_message = "";
      this->type = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _error_message_type error_message;
  using _type_type =
    uint8_t;
  _type_type type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FATAL =
    0u;
  static constexpr uint8_t NON_FATAL =
    1u;

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::Error_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::Error_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::Error_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::Error_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::Error_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::Error_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::Error_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::Error_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::Error_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::Error_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__Error
    std::shared_ptr<march_shared_msgs::msg::Error_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__Error
    std::shared_ptr<march_shared_msgs::msg::Error_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Error_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const Error_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Error_

// alias to use template instance with default allocator
using Error =
  march_shared_msgs::msg::Error_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t Error_<ContainerAllocator>::FATAL;
template<typename ContainerAllocator>
constexpr uint8_t Error_<ContainerAllocator>::NON_FATAL;

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__ERROR__STRUCT_HPP_
