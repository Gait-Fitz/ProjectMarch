// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/LowVoltageState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__LOW_VOLTAGE_STATE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__LOW_VOLTAGE_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__LowVoltageState __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__LowVoltageState __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LowVoltageState_
{
  using Type = LowVoltageState_<ContainerAllocator>;

  explicit LowVoltageState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lv1_current = 0.0f;
      this->lv2_current = 0.0f;
      this->lv1_ok = 0ul;
      this->lv2_ok = 0ul;
    }
  }

  explicit LowVoltageState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lv1_current = 0.0f;
      this->lv2_current = 0.0f;
      this->lv1_ok = 0ul;
      this->lv2_ok = 0ul;
    }
  }

  // field types and members
  using _lv1_current_type =
    float;
  _lv1_current_type lv1_current;
  using _lv2_current_type =
    float;
  _lv2_current_type lv2_current;
  using _lv1_ok_type =
    uint32_t;
  _lv1_ok_type lv1_ok;
  using _lv2_ok_type =
    uint32_t;
  _lv2_ok_type lv2_ok;

  // setters for named parameter idiom
  Type & set__lv1_current(
    const float & _arg)
  {
    this->lv1_current = _arg;
    return *this;
  }
  Type & set__lv2_current(
    const float & _arg)
  {
    this->lv2_current = _arg;
    return *this;
  }
  Type & set__lv1_ok(
    const uint32_t & _arg)
  {
    this->lv1_ok = _arg;
    return *this;
  }
  Type & set__lv2_ok(
    const uint32_t & _arg)
  {
    this->lv2_ok = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::LowVoltageState_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::LowVoltageState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::LowVoltageState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::LowVoltageState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::LowVoltageState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::LowVoltageState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::LowVoltageState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::LowVoltageState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::LowVoltageState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::LowVoltageState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__LowVoltageState
    std::shared_ptr<march_shared_msgs::msg::LowVoltageState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__LowVoltageState
    std::shared_ptr<march_shared_msgs::msg::LowVoltageState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LowVoltageState_ & other) const
  {
    if (this->lv1_current != other.lv1_current) {
      return false;
    }
    if (this->lv2_current != other.lv2_current) {
      return false;
    }
    if (this->lv1_ok != other.lv1_ok) {
      return false;
    }
    if (this->lv2_ok != other.lv2_ok) {
      return false;
    }
    return true;
  }
  bool operator!=(const LowVoltageState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LowVoltageState_

// alias to use template instance with default allocator
using LowVoltageState =
  march_shared_msgs::msg::LowVoltageState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__LOW_VOLTAGE_STATE__STRUCT_HPP_
