// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/MpcJoint.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'estimation'
// Member 'reference'
#include "march_shared_msgs/msg/detail/mpc_state_vectors__struct.hpp"
// Member 'tuning'
#include "march_shared_msgs/msg/detail/mpc_tuning__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__MpcJoint __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__MpcJoint __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MpcJoint_
{
  using Type = MpcJoint_<ContainerAllocator>;

  explicit MpcJoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : estimation(_init),
    reference(_init),
    tuning(_init)
  {
    (void)_init;
  }

  explicit MpcJoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : estimation(_alloc, _init),
    reference(_alloc, _init),
    tuning(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _estimation_type =
    march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator>;
  _estimation_type estimation;
  using _reference_type =
    march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator>;
  _reference_type reference;
  using _tuning_type =
    march_shared_msgs::msg::MpcTuning_<ContainerAllocator>;
  _tuning_type tuning;

  // setters for named parameter idiom
  Type & set__estimation(
    const march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator> & _arg)
  {
    this->estimation = _arg;
    return *this;
  }
  Type & set__reference(
    const march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator> & _arg)
  {
    this->reference = _arg;
    return *this;
  }
  Type & set__tuning(
    const march_shared_msgs::msg::MpcTuning_<ContainerAllocator> & _arg)
  {
    this->tuning = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::MpcJoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::MpcJoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcJoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcJoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcJoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcJoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcJoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcJoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcJoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcJoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__MpcJoint
    std::shared_ptr<march_shared_msgs::msg::MpcJoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__MpcJoint
    std::shared_ptr<march_shared_msgs::msg::MpcJoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MpcJoint_ & other) const
  {
    if (this->estimation != other.estimation) {
      return false;
    }
    if (this->reference != other.reference) {
      return false;
    }
    if (this->tuning != other.tuning) {
      return false;
    }
    return true;
  }
  bool operator!=(const MpcJoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MpcJoint_

// alias to use template instance with default allocator
using MpcJoint =
  march_shared_msgs::msg::MpcJoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_JOINT__STRUCT_HPP_
