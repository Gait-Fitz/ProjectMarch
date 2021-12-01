// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/GaitInstruction.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION__STRUCT_HPP_

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
# define DEPRECATED__march_shared_msgs__msg__GaitInstruction __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__GaitInstruction __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GaitInstruction_
{
  using Type = GaitInstruction_<ContainerAllocator>;

  explicit GaitInstruction_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->gait_name = "";
      this->id = "";
    }
  }

  explicit GaitInstruction_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    gait_name(_alloc),
    id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->gait_name = "";
      this->id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _type_type =
    int8_t;
  _type_type type;
  using _gait_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _gait_name_type gait_name;
  using _id_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _id_type id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__gait_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->gait_name = _arg;
    return *this;
  }
  Type & set__id(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t UNKNOWN =
    -1;
  static constexpr int8_t STOP =
    0;
  static constexpr int8_t GAIT =
    1;
  static constexpr int8_t PAUSE =
    2;
  static constexpr int8_t CONTINUE =
    3;
  static constexpr int8_t INCREMENT_STEP_SIZE =
    4;
  static constexpr int8_t DECREMENT_STEP_SIZE =
    5;

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::GaitInstruction_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::GaitInstruction_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::GaitInstruction_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::GaitInstruction_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::GaitInstruction_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::GaitInstruction_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::GaitInstruction_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::GaitInstruction_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::GaitInstruction_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::GaitInstruction_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__GaitInstruction
    std::shared_ptr<march_shared_msgs::msg::GaitInstruction_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__GaitInstruction
    std::shared_ptr<march_shared_msgs::msg::GaitInstruction_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GaitInstruction_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->gait_name != other.gait_name) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GaitInstruction_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GaitInstruction_

// alias to use template instance with default allocator
using GaitInstruction =
  march_shared_msgs::msg::GaitInstruction_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t GaitInstruction_<ContainerAllocator>::UNKNOWN;
template<typename ContainerAllocator>
constexpr int8_t GaitInstruction_<ContainerAllocator>::STOP;
template<typename ContainerAllocator>
constexpr int8_t GaitInstruction_<ContainerAllocator>::GAIT;
template<typename ContainerAllocator>
constexpr int8_t GaitInstruction_<ContainerAllocator>::PAUSE;
template<typename ContainerAllocator>
constexpr int8_t GaitInstruction_<ContainerAllocator>::CONTINUE;
template<typename ContainerAllocator>
constexpr int8_t GaitInstruction_<ContainerAllocator>::INCREMENT_STEP_SIZE;
template<typename ContainerAllocator>
constexpr int8_t GaitInstruction_<ContainerAllocator>::DECREMENT_STEP_SIZE;

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION__STRUCT_HPP_
