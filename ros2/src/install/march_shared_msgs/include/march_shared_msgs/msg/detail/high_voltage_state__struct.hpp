// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/HighVoltageState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__HIGH_VOLTAGE_STATE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__HIGH_VOLTAGE_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__HighVoltageState __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__HighVoltageState __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HighVoltageState_
{
  using Type = HighVoltageState_<ContainerAllocator>;

  explicit HighVoltageState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_current = 0.0f;
      this->hv1_current = 0.0f;
      this->hv2_current = 0.0f;
      this->hv3_current = 0.0f;
      this->hv4_current = 0.0f;
    }
  }

  explicit HighVoltageState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_current = 0.0f;
      this->hv1_current = 0.0f;
      this->hv2_current = 0.0f;
      this->hv3_current = 0.0f;
      this->hv4_current = 0.0f;
    }
  }

  // field types and members
  using _total_current_type =
    float;
  _total_current_type total_current;
  using _hv1_current_type =
    float;
  _hv1_current_type hv1_current;
  using _hv2_current_type =
    float;
  _hv2_current_type hv2_current;
  using _hv3_current_type =
    float;
  _hv3_current_type hv3_current;
  using _hv4_current_type =
    float;
  _hv4_current_type hv4_current;

  // setters for named parameter idiom
  Type & set__total_current(
    const float & _arg)
  {
    this->total_current = _arg;
    return *this;
  }
  Type & set__hv1_current(
    const float & _arg)
  {
    this->hv1_current = _arg;
    return *this;
  }
  Type & set__hv2_current(
    const float & _arg)
  {
    this->hv2_current = _arg;
    return *this;
  }
  Type & set__hv3_current(
    const float & _arg)
  {
    this->hv3_current = _arg;
    return *this;
  }
  Type & set__hv4_current(
    const float & _arg)
  {
    this->hv4_current = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::HighVoltageState_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::HighVoltageState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::HighVoltageState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::HighVoltageState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::HighVoltageState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::HighVoltageState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::HighVoltageState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::HighVoltageState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::HighVoltageState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::HighVoltageState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__HighVoltageState
    std::shared_ptr<march_shared_msgs::msg::HighVoltageState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__HighVoltageState
    std::shared_ptr<march_shared_msgs::msg::HighVoltageState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HighVoltageState_ & other) const
  {
    if (this->total_current != other.total_current) {
      return false;
    }
    if (this->hv1_current != other.hv1_current) {
      return false;
    }
    if (this->hv2_current != other.hv2_current) {
      return false;
    }
    if (this->hv3_current != other.hv3_current) {
      return false;
    }
    if (this->hv4_current != other.hv4_current) {
      return false;
    }
    return true;
  }
  bool operator!=(const HighVoltageState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HighVoltageState_

// alias to use template instance with default allocator
using HighVoltageState =
  march_shared_msgs::msg::HighVoltageState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__HIGH_VOLTAGE_STATE__STRUCT_HPP_
