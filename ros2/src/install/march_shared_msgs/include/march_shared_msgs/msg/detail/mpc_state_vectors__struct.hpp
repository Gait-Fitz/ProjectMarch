// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/MpcStateVectors.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_STATE_VECTORS__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_STATE_VECTORS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'states'
// Member 'inputs'
#include "march_shared_msgs/msg/detail/mpc_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__MpcStateVectors __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__MpcStateVectors __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MpcStateVectors_
{
  using Type = MpcStateVectors_<ContainerAllocator>;

  explicit MpcStateVectors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MpcStateVectors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _states_type =
    std::vector<march_shared_msgs::msg::MpcArray_<ContainerAllocator>, typename ContainerAllocator::template rebind<march_shared_msgs::msg::MpcArray_<ContainerAllocator>>::other>;
  _states_type states;
  using _inputs_type =
    std::vector<march_shared_msgs::msg::MpcArray_<ContainerAllocator>, typename ContainerAllocator::template rebind<march_shared_msgs::msg::MpcArray_<ContainerAllocator>>::other>;
  _inputs_type inputs;

  // setters for named parameter idiom
  Type & set__states(
    const std::vector<march_shared_msgs::msg::MpcArray_<ContainerAllocator>, typename ContainerAllocator::template rebind<march_shared_msgs::msg::MpcArray_<ContainerAllocator>>::other> & _arg)
  {
    this->states = _arg;
    return *this;
  }
  Type & set__inputs(
    const std::vector<march_shared_msgs::msg::MpcArray_<ContainerAllocator>, typename ContainerAllocator::template rebind<march_shared_msgs::msg::MpcArray_<ContainerAllocator>>::other> & _arg)
  {
    this->inputs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__MpcStateVectors
    std::shared_ptr<march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__MpcStateVectors
    std::shared_ptr<march_shared_msgs::msg::MpcStateVectors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MpcStateVectors_ & other) const
  {
    if (this->states != other.states) {
      return false;
    }
    if (this->inputs != other.inputs) {
      return false;
    }
    return true;
  }
  bool operator!=(const MpcStateVectors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MpcStateVectors_

// alias to use template instance with default allocator
using MpcStateVectors =
  march_shared_msgs::msg::MpcStateVectors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_STATE_VECTORS__STRUCT_HPP_
