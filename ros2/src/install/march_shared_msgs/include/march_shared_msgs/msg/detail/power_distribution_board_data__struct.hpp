// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/PowerDistributionBoardData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__STRUCT_HPP_

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
// Member 'hv_state'
#include "march_shared_msgs/msg/detail/high_voltage_state__struct.hpp"
// Member 'lv_state'
#include "march_shared_msgs/msg/detail/low_voltage_state__struct.hpp"
// Member 'battery_state'
#include "march_shared_msgs/msg/detail/battery_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__PowerDistributionBoardData __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__PowerDistributionBoardData __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PowerDistributionBoardData_
{
  using Type = PowerDistributionBoardData_<ContainerAllocator>;

  explicit PowerDistributionBoardData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    hv_state(_init),
    lv_state(_init),
    battery_state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->emergency_button_state = 0ul;
      this->pdb_current = 0.0f;
      this->stop_button_state = 0ul;
    }
  }

  explicit PowerDistributionBoardData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    hv_state(_alloc, _init),
    lv_state(_alloc, _init),
    battery_state(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->emergency_button_state = 0ul;
      this->pdb_current = 0.0f;
      this->stop_button_state = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _emergency_button_state_type =
    uint32_t;
  _emergency_button_state_type emergency_button_state;
  using _pdb_current_type =
    float;
  _pdb_current_type pdb_current;
  using _hv_state_type =
    march_shared_msgs::msg::HighVoltageState_<ContainerAllocator>;
  _hv_state_type hv_state;
  using _stop_button_state_type =
    uint32_t;
  _stop_button_state_type stop_button_state;
  using _lv_state_type =
    march_shared_msgs::msg::LowVoltageState_<ContainerAllocator>;
  _lv_state_type lv_state;
  using _battery_state_type =
    march_shared_msgs::msg::BatteryState_<ContainerAllocator>;
  _battery_state_type battery_state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__emergency_button_state(
    const uint32_t & _arg)
  {
    this->emergency_button_state = _arg;
    return *this;
  }
  Type & set__pdb_current(
    const float & _arg)
  {
    this->pdb_current = _arg;
    return *this;
  }
  Type & set__hv_state(
    const march_shared_msgs::msg::HighVoltageState_<ContainerAllocator> & _arg)
  {
    this->hv_state = _arg;
    return *this;
  }
  Type & set__stop_button_state(
    const uint32_t & _arg)
  {
    this->stop_button_state = _arg;
    return *this;
  }
  Type & set__lv_state(
    const march_shared_msgs::msg::LowVoltageState_<ContainerAllocator> & _arg)
  {
    this->lv_state = _arg;
    return *this;
  }
  Type & set__battery_state(
    const march_shared_msgs::msg::BatteryState_<ContainerAllocator> & _arg)
  {
    this->battery_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__PowerDistributionBoardData
    std::shared_ptr<march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__PowerDistributionBoardData
    std::shared_ptr<march_shared_msgs::msg::PowerDistributionBoardData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PowerDistributionBoardData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->emergency_button_state != other.emergency_button_state) {
      return false;
    }
    if (this->pdb_current != other.pdb_current) {
      return false;
    }
    if (this->hv_state != other.hv_state) {
      return false;
    }
    if (this->stop_button_state != other.stop_button_state) {
      return false;
    }
    if (this->lv_state != other.lv_state) {
      return false;
    }
    if (this->battery_state != other.battery_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const PowerDistributionBoardData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PowerDistributionBoardData_

// alias to use template instance with default allocator
using PowerDistributionBoardData =
  march_shared_msgs::msg::PowerDistributionBoardData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__STRUCT_HPP_
