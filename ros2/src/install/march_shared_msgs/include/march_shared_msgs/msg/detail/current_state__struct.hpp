// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/CurrentState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_STATE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_STATE__STRUCT_HPP_

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
# define DEPRECATED__march_shared_msgs__msg__CurrentState __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__CurrentState __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CurrentState_
{
  using Type = CurrentState_<ContainerAllocator>;

  explicit CurrentState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->state_type = 0;
    }
  }

  explicit CurrentState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->state_type = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _state_type state;
  using _state_type_type =
    int8_t;
  _state_type_type state_type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__state_type(
    const int8_t & _arg)
  {
    this->state_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t IDLE =
    0;
  static constexpr int8_t GAIT =
    1;

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::CurrentState_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::CurrentState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::CurrentState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::CurrentState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::CurrentState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::CurrentState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::CurrentState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::CurrentState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::CurrentState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::CurrentState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__CurrentState
    std::shared_ptr<march_shared_msgs::msg::CurrentState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__CurrentState
    std::shared_ptr<march_shared_msgs::msg::CurrentState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CurrentState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->state_type != other.state_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const CurrentState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CurrentState_

// alias to use template instance with default allocator
using CurrentState =
  march_shared_msgs::msg::CurrentState_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t CurrentState_<ContainerAllocator>::IDLE;
template<typename ContainerAllocator>
constexpr int8_t CurrentState_<ContainerAllocator>::GAIT;

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__CURRENT_STATE__STRUCT_HPP_
