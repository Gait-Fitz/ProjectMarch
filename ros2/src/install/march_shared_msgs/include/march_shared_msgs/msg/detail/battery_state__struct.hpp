// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/BatteryState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__BATTERY_STATE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__BATTERY_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__BatteryState __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__BatteryState __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BatteryState_
{
  using Type = BatteryState_<ContainerAllocator>;

  explicit BatteryState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->percentage = 0.0f;
      this->voltage = 0.0f;
      this->temperature = 0.0f;
    }
  }

  explicit BatteryState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->percentage = 0.0f;
      this->voltage = 0.0f;
      this->temperature = 0.0f;
    }
  }

  // field types and members
  using _percentage_type =
    float;
  _percentage_type percentage;
  using _voltage_type =
    float;
  _voltage_type voltage;
  using _temperature_type =
    float;
  _temperature_type temperature;

  // setters for named parameter idiom
  Type & set__percentage(
    const float & _arg)
  {
    this->percentage = _arg;
    return *this;
  }
  Type & set__voltage(
    const float & _arg)
  {
    this->voltage = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::BatteryState_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::BatteryState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::BatteryState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::BatteryState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::BatteryState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::BatteryState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::BatteryState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::BatteryState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::BatteryState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::BatteryState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__BatteryState
    std::shared_ptr<march_shared_msgs::msg::BatteryState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__BatteryState
    std::shared_ptr<march_shared_msgs::msg::BatteryState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BatteryState_ & other) const
  {
    if (this->percentage != other.percentage) {
      return false;
    }
    if (this->voltage != other.voltage) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const BatteryState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BatteryState_

// alias to use template instance with default allocator
using BatteryState =
  march_shared_msgs::msg::BatteryState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__BATTERY_STATE__STRUCT_HPP_
