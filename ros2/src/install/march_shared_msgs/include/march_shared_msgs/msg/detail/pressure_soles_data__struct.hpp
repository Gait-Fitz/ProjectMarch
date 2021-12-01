// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/PressureSolesData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__STRUCT_HPP_

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
// Member 'left_foot'
// Member 'right_foot'
#include "march_shared_msgs/msg/detail/pressure_sole_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__PressureSolesData __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__PressureSolesData __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PressureSolesData_
{
  using Type = PressureSolesData_<ContainerAllocator>;

  explicit PressureSolesData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    left_foot(_init),
    right_foot(_init)
  {
    (void)_init;
  }

  explicit PressureSolesData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    left_foot(_alloc, _init),
    right_foot(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _left_foot_type =
    march_shared_msgs::msg::PressureSoleData_<ContainerAllocator>;
  _left_foot_type left_foot;
  using _right_foot_type =
    march_shared_msgs::msg::PressureSoleData_<ContainerAllocator>;
  _right_foot_type right_foot;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__left_foot(
    const march_shared_msgs::msg::PressureSoleData_<ContainerAllocator> & _arg)
  {
    this->left_foot = _arg;
    return *this;
  }
  Type & set__right_foot(
    const march_shared_msgs::msg::PressureSoleData_<ContainerAllocator> & _arg)
  {
    this->right_foot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::PressureSolesData_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::PressureSolesData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::PressureSolesData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::PressureSolesData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::PressureSolesData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::PressureSolesData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::PressureSolesData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::PressureSolesData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::PressureSolesData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::PressureSolesData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__PressureSolesData
    std::shared_ptr<march_shared_msgs::msg::PressureSolesData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__PressureSolesData
    std::shared_ptr<march_shared_msgs::msg::PressureSolesData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressureSolesData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->left_foot != other.left_foot) {
      return false;
    }
    if (this->right_foot != other.right_foot) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressureSolesData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressureSolesData_

// alias to use template instance with default allocator
using PressureSolesData =
  march_shared_msgs::msg::PressureSolesData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLES_DATA__STRUCT_HPP_
