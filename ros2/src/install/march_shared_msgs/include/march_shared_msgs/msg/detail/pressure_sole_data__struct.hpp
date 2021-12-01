// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/PressureSoleData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__PressureSoleData __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__PressureSoleData __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PressureSoleData_
{
  using Type = PressureSoleData_<ContainerAllocator>;

  explicit PressureSoleData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->side = "";
      this->heel_right = 0.0f;
      this->heel_left = 0.0f;
      this->met1 = 0.0f;
      this->hallux = 0.0f;
      this->met3 = 0.0f;
      this->toes = 0.0f;
      this->met5 = 0.0f;
      this->arch = 0.0f;
    }
  }

  explicit PressureSoleData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : side(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->side = "";
      this->heel_right = 0.0f;
      this->heel_left = 0.0f;
      this->met1 = 0.0f;
      this->hallux = 0.0f;
      this->met3 = 0.0f;
      this->toes = 0.0f;
      this->met5 = 0.0f;
      this->arch = 0.0f;
    }
  }

  // field types and members
  using _side_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _side_type side;
  using _heel_right_type =
    float;
  _heel_right_type heel_right;
  using _heel_left_type =
    float;
  _heel_left_type heel_left;
  using _met1_type =
    float;
  _met1_type met1;
  using _hallux_type =
    float;
  _hallux_type hallux;
  using _met3_type =
    float;
  _met3_type met3;
  using _toes_type =
    float;
  _toes_type toes;
  using _met5_type =
    float;
  _met5_type met5;
  using _arch_type =
    float;
  _arch_type arch;

  // setters for named parameter idiom
  Type & set__side(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->side = _arg;
    return *this;
  }
  Type & set__heel_right(
    const float & _arg)
  {
    this->heel_right = _arg;
    return *this;
  }
  Type & set__heel_left(
    const float & _arg)
  {
    this->heel_left = _arg;
    return *this;
  }
  Type & set__met1(
    const float & _arg)
  {
    this->met1 = _arg;
    return *this;
  }
  Type & set__hallux(
    const float & _arg)
  {
    this->hallux = _arg;
    return *this;
  }
  Type & set__met3(
    const float & _arg)
  {
    this->met3 = _arg;
    return *this;
  }
  Type & set__toes(
    const float & _arg)
  {
    this->toes = _arg;
    return *this;
  }
  Type & set__met5(
    const float & _arg)
  {
    this->met5 = _arg;
    return *this;
  }
  Type & set__arch(
    const float & _arg)
  {
    this->arch = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::PressureSoleData_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::PressureSoleData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::PressureSoleData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::PressureSoleData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::PressureSoleData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::PressureSoleData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::PressureSoleData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::PressureSoleData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::PressureSoleData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::PressureSoleData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__PressureSoleData
    std::shared_ptr<march_shared_msgs::msg::PressureSoleData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__PressureSoleData
    std::shared_ptr<march_shared_msgs::msg::PressureSoleData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressureSoleData_ & other) const
  {
    if (this->side != other.side) {
      return false;
    }
    if (this->heel_right != other.heel_right) {
      return false;
    }
    if (this->heel_left != other.heel_left) {
      return false;
    }
    if (this->met1 != other.met1) {
      return false;
    }
    if (this->hallux != other.hallux) {
      return false;
    }
    if (this->met3 != other.met3) {
      return false;
    }
    if (this->toes != other.toes) {
      return false;
    }
    if (this->met5 != other.met5) {
      return false;
    }
    if (this->arch != other.arch) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressureSoleData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressureSoleData_

// alias to use template instance with default allocator
using PressureSoleData =
  march_shared_msgs::msg::PressureSoleData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__PRESSURE_SOLE_DATA__STRUCT_HPP_
