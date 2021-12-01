// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/GaitParameters.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__GaitParameters __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__GaitParameters __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GaitParameters_
{
  using Type = GaitParameters_<ContainerAllocator>;

  explicit GaitParameters_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->first_parameter = 0.0;
      this->second_parameter = 0.0;
      this->side_step_parameter = 0.0;
    }
  }

  explicit GaitParameters_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->first_parameter = 0.0;
      this->second_parameter = 0.0;
      this->side_step_parameter = 0.0;
    }
  }

  // field types and members
  using _first_parameter_type =
    double;
  _first_parameter_type first_parameter;
  using _second_parameter_type =
    double;
  _second_parameter_type second_parameter;
  using _side_step_parameter_type =
    double;
  _side_step_parameter_type side_step_parameter;

  // setters for named parameter idiom
  Type & set__first_parameter(
    const double & _arg)
  {
    this->first_parameter = _arg;
    return *this;
  }
  Type & set__second_parameter(
    const double & _arg)
  {
    this->second_parameter = _arg;
    return *this;
  }
  Type & set__side_step_parameter(
    const double & _arg)
  {
    this->side_step_parameter = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::GaitParameters_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::GaitParameters_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::GaitParameters_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::GaitParameters_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::GaitParameters_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::GaitParameters_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::GaitParameters_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::GaitParameters_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::GaitParameters_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::GaitParameters_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__GaitParameters
    std::shared_ptr<march_shared_msgs::msg::GaitParameters_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__GaitParameters
    std::shared_ptr<march_shared_msgs::msg::GaitParameters_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GaitParameters_ & other) const
  {
    if (this->first_parameter != other.first_parameter) {
      return false;
    }
    if (this->second_parameter != other.second_parameter) {
      return false;
    }
    if (this->side_step_parameter != other.side_step_parameter) {
      return false;
    }
    return true;
  }
  bool operator!=(const GaitParameters_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GaitParameters_

// alias to use template instance with default allocator
using GaitParameters =
  march_shared_msgs::msg::GaitParameters_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__STRUCT_HPP_
