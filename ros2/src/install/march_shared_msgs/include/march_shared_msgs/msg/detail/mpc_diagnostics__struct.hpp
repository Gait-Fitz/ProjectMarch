// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/MpcDiagnostics.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_DIAGNOSTICS__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_DIAGNOSTICS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__MpcDiagnostics __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__MpcDiagnostics __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MpcDiagnostics_
{
  using Type = MpcDiagnostics_<ContainerAllocator>;

  explicit MpcDiagnostics_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->preparation_time = 0.0;
      this->feedback_time = 0.0;
      this->total_time = 0.0;
      this->preparation_status = 0l;
      this->feedback_status = 0l;
      this->cost = 0.0f;
    }
  }

  explicit MpcDiagnostics_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->preparation_time = 0.0;
      this->feedback_time = 0.0;
      this->total_time = 0.0;
      this->preparation_status = 0l;
      this->feedback_status = 0l;
      this->cost = 0.0f;
    }
  }

  // field types and members
  using _preparation_time_type =
    double;
  _preparation_time_type preparation_time;
  using _feedback_time_type =
    double;
  _feedback_time_type feedback_time;
  using _total_time_type =
    double;
  _total_time_type total_time;
  using _preparation_status_type =
    int32_t;
  _preparation_status_type preparation_status;
  using _feedback_status_type =
    int32_t;
  _feedback_status_type feedback_status;
  using _cost_type =
    float;
  _cost_type cost;

  // setters for named parameter idiom
  Type & set__preparation_time(
    const double & _arg)
  {
    this->preparation_time = _arg;
    return *this;
  }
  Type & set__feedback_time(
    const double & _arg)
  {
    this->feedback_time = _arg;
    return *this;
  }
  Type & set__total_time(
    const double & _arg)
  {
    this->total_time = _arg;
    return *this;
  }
  Type & set__preparation_status(
    const int32_t & _arg)
  {
    this->preparation_status = _arg;
    return *this;
  }
  Type & set__feedback_status(
    const int32_t & _arg)
  {
    this->feedback_status = _arg;
    return *this;
  }
  Type & set__cost(
    const float & _arg)
  {
    this->cost = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__MpcDiagnostics
    std::shared_ptr<march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__MpcDiagnostics
    std::shared_ptr<march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MpcDiagnostics_ & other) const
  {
    if (this->preparation_time != other.preparation_time) {
      return false;
    }
    if (this->feedback_time != other.feedback_time) {
      return false;
    }
    if (this->total_time != other.total_time) {
      return false;
    }
    if (this->preparation_status != other.preparation_status) {
      return false;
    }
    if (this->feedback_status != other.feedback_status) {
      return false;
    }
    if (this->cost != other.cost) {
      return false;
    }
    return true;
  }
  bool operator!=(const MpcDiagnostics_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MpcDiagnostics_

// alias to use template instance with default allocator
using MpcDiagnostics =
  march_shared_msgs::msg::MpcDiagnostics_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_DIAGNOSTICS__STRUCT_HPP_
