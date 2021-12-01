// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/MpcArray.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_ARRAY__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_ARRAY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__MpcArray __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__MpcArray __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MpcArray_
{
  using Type = MpcArray_<ContainerAllocator>;

  explicit MpcArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MpcArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _array_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _array_type array;

  // setters for named parameter idiom
  Type & set__array(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->array = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::MpcArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::MpcArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__MpcArray
    std::shared_ptr<march_shared_msgs::msg::MpcArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__MpcArray
    std::shared_ptr<march_shared_msgs::msg::MpcArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MpcArray_ & other) const
  {
    if (this->array != other.array) {
      return false;
    }
    return true;
  }
  bool operator!=(const MpcArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MpcArray_

// alias to use template instance with default allocator
using MpcArray =
  march_shared_msgs::msg::MpcArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_ARRAY__STRUCT_HPP_
