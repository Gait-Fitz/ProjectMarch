// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/MpcTuning.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_TUNING__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_TUNING__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__MpcTuning __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__MpcTuning __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MpcTuning_
{
  using Type = MpcTuning_<ContainerAllocator>;

  explicit MpcTuning_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->horizon = 0l;
    }
  }

  explicit MpcTuning_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->horizon = 0l;
    }
  }

  // field types and members
  using _q_weights_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _q_weights_type q_weights;
  using _r_weights_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _r_weights_type r_weights;
  using _horizon_type =
    int32_t;
  _horizon_type horizon;

  // setters for named parameter idiom
  Type & set__q_weights(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->q_weights = _arg;
    return *this;
  }
  Type & set__r_weights(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->r_weights = _arg;
    return *this;
  }
  Type & set__horizon(
    const int32_t & _arg)
  {
    this->horizon = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::MpcTuning_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::MpcTuning_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcTuning_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcTuning_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcTuning_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcTuning_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcTuning_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcTuning_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcTuning_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcTuning_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__MpcTuning
    std::shared_ptr<march_shared_msgs::msg::MpcTuning_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__MpcTuning
    std::shared_ptr<march_shared_msgs::msg::MpcTuning_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MpcTuning_ & other) const
  {
    if (this->q_weights != other.q_weights) {
      return false;
    }
    if (this->r_weights != other.r_weights) {
      return false;
    }
    if (this->horizon != other.horizon) {
      return false;
    }
    return true;
  }
  bool operator!=(const MpcTuning_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MpcTuning_

// alias to use template instance with default allocator
using MpcTuning =
  march_shared_msgs::msg::MpcTuning_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_TUNING__STRUCT_HPP_
