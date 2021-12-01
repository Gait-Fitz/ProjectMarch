// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/MotorControllerState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MOTOR_CONTROLLER_STATE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MOTOR_CONTROLLER_STATE__STRUCT_HPP_

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
# define DEPRECATED__march_shared_msgs__msg__MotorControllerState __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__MotorControllerState __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorControllerState_
{
  using Type = MotorControllerState_<ContainerAllocator>;

  explicit MotorControllerState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MotorControllerState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _joint_names_type joint_names;
  using _error_status_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _error_status_type error_status;
  using _operational_state_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _operational_state_type operational_state;
  using _motor_current_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _motor_current_type motor_current;
  using _motor_voltage_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _motor_voltage_type motor_voltage;
  using _temperature_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _temperature_type temperature;
  using _absolute_position_iu_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _absolute_position_iu_type absolute_position_iu;
  using _incremental_position_iu_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _incremental_position_iu_type incremental_position_iu;
  using _absolute_velocity_iu_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _absolute_velocity_iu_type absolute_velocity_iu;
  using _incremental_velocity_iu_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _incremental_velocity_iu_type incremental_velocity_iu;
  using _absolute_position_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _absolute_position_type absolute_position;
  using _incremental_position_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _incremental_position_type incremental_position;
  using _absolute_velocity_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _absolute_velocity_type absolute_velocity;
  using _incremental_velocity_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _incremental_velocity_type incremental_velocity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->joint_names = _arg;
    return *this;
  }
  Type & set__error_status(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->error_status = _arg;
    return *this;
  }
  Type & set__operational_state(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->operational_state = _arg;
    return *this;
  }
  Type & set__motor_current(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->motor_current = _arg;
    return *this;
  }
  Type & set__motor_voltage(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->motor_voltage = _arg;
    return *this;
  }
  Type & set__temperature(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__absolute_position_iu(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->absolute_position_iu = _arg;
    return *this;
  }
  Type & set__incremental_position_iu(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->incremental_position_iu = _arg;
    return *this;
  }
  Type & set__absolute_velocity_iu(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->absolute_velocity_iu = _arg;
    return *this;
  }
  Type & set__incremental_velocity_iu(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->incremental_velocity_iu = _arg;
    return *this;
  }
  Type & set__absolute_position(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->absolute_position = _arg;
    return *this;
  }
  Type & set__incremental_position(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->incremental_position = _arg;
    return *this;
  }
  Type & set__absolute_velocity(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->absolute_velocity = _arg;
    return *this;
  }
  Type & set__incremental_velocity(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->incremental_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::MotorControllerState_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::MotorControllerState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MotorControllerState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MotorControllerState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MotorControllerState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MotorControllerState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MotorControllerState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MotorControllerState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MotorControllerState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MotorControllerState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__MotorControllerState
    std::shared_ptr<march_shared_msgs::msg::MotorControllerState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__MotorControllerState
    std::shared_ptr<march_shared_msgs::msg::MotorControllerState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorControllerState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_names != other.joint_names) {
      return false;
    }
    if (this->error_status != other.error_status) {
      return false;
    }
    if (this->operational_state != other.operational_state) {
      return false;
    }
    if (this->motor_current != other.motor_current) {
      return false;
    }
    if (this->motor_voltage != other.motor_voltage) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->absolute_position_iu != other.absolute_position_iu) {
      return false;
    }
    if (this->incremental_position_iu != other.incremental_position_iu) {
      return false;
    }
    if (this->absolute_velocity_iu != other.absolute_velocity_iu) {
      return false;
    }
    if (this->incremental_velocity_iu != other.incremental_velocity_iu) {
      return false;
    }
    if (this->absolute_position != other.absolute_position) {
      return false;
    }
    if (this->incremental_position != other.incremental_position) {
      return false;
    }
    if (this->absolute_velocity != other.absolute_velocity) {
      return false;
    }
    if (this->incremental_velocity != other.incremental_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorControllerState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorControllerState_

// alias to use template instance with default allocator
using MotorControllerState =
  march_shared_msgs::msg::MotorControllerState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MOTOR_CONTROLLER_STATE__STRUCT_HPP_
