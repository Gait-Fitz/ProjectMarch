// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/PressureSole.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__STRUCT_HPP_

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
// Member 'pressure_soles_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__PressureSole __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__PressureSole __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PressureSole_
{
  using Type = PressureSole_<ContainerAllocator>;

  explicit PressureSole_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pressure_soles_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_force_left = 0.0f;
      this->total_force_right = 0.0f;
    }
  }

  explicit PressureSole_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pressure_soles_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_force_left = 0.0f;
      this->total_force_right = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pressure_soles_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _pressure_soles_time_type pressure_soles_time;
  using _total_force_left_type =
    float;
  _total_force_left_type total_force_left;
  using _total_force_right_type =
    float;
  _total_force_right_type total_force_right;
  using _pressure_left_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _pressure_left_type pressure_left;
  using _pressure_right_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _pressure_right_type pressure_right;
  using _cop_left_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _cop_left_type cop_left;
  using _cop_right_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _cop_right_type cop_right;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pressure_soles_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->pressure_soles_time = _arg;
    return *this;
  }
  Type & set__total_force_left(
    const float & _arg)
  {
    this->total_force_left = _arg;
    return *this;
  }
  Type & set__total_force_right(
    const float & _arg)
  {
    this->total_force_right = _arg;
    return *this;
  }
  Type & set__pressure_left(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->pressure_left = _arg;
    return *this;
  }
  Type & set__pressure_right(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->pressure_right = _arg;
    return *this;
  }
  Type & set__cop_left(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->cop_left = _arg;
    return *this;
  }
  Type & set__cop_right(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->cop_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::PressureSole_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::PressureSole_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::PressureSole_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::PressureSole_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::PressureSole_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::PressureSole_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::PressureSole_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::PressureSole_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::PressureSole_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::PressureSole_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__PressureSole
    std::shared_ptr<march_shared_msgs::msg::PressureSole_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__PressureSole
    std::shared_ptr<march_shared_msgs::msg::PressureSole_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressureSole_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pressure_soles_time != other.pressure_soles_time) {
      return false;
    }
    if (this->total_force_left != other.total_force_left) {
      return false;
    }
    if (this->total_force_right != other.total_force_right) {
      return false;
    }
    if (this->pressure_left != other.pressure_left) {
      return false;
    }
    if (this->pressure_right != other.pressure_right) {
      return false;
    }
    if (this->cop_left != other.cop_left) {
      return false;
    }
    if (this->cop_right != other.cop_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressureSole_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressureSole_

// alias to use template instance with default allocator
using PressureSole =
  march_shared_msgs::msg::PressureSole_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE__STRUCT_HPP_
