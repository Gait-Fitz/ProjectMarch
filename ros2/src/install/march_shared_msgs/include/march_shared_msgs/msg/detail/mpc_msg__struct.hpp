// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/MpcMsg.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MPC_MSG__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MPC_MSG__STRUCT_HPP_

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
// Member 'diagnostics'
#include "march_shared_msgs/msg/detail/mpc_diagnostics__struct.hpp"
// Member 'joint'
#include "march_shared_msgs/msg/detail/mpc_joint__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__MpcMsg __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__MpcMsg __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MpcMsg_
{
  using Type = MpcMsg_<ContainerAllocator>;

  explicit MpcMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    diagnostics(_init)
  {
    (void)_init;
  }

  explicit MpcMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    diagnostics(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _diagnostics_type =
    march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator>;
  _diagnostics_type diagnostics;
  using _joint_type =
    std::vector<march_shared_msgs::msg::MpcJoint_<ContainerAllocator>, typename ContainerAllocator::template rebind<march_shared_msgs::msg::MpcJoint_<ContainerAllocator>>::other>;
  _joint_type joint;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__diagnostics(
    const march_shared_msgs::msg::MpcDiagnostics_<ContainerAllocator> & _arg)
  {
    this->diagnostics = _arg;
    return *this;
  }
  Type & set__joint(
    const std::vector<march_shared_msgs::msg::MpcJoint_<ContainerAllocator>, typename ContainerAllocator::template rebind<march_shared_msgs::msg::MpcJoint_<ContainerAllocator>>::other> & _arg)
  {
    this->joint = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::MpcMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::MpcMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::MpcMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::MpcMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::MpcMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::MpcMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__MpcMsg
    std::shared_ptr<march_shared_msgs::msg::MpcMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__MpcMsg
    std::shared_ptr<march_shared_msgs::msg::MpcMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MpcMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->diagnostics != other.diagnostics) {
      return false;
    }
    if (this->joint != other.joint) {
      return false;
    }
    return true;
  }
  bool operator!=(const MpcMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MpcMsg_

// alias to use template instance with default allocator
using MpcMsg =
  march_shared_msgs::msg::MpcMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MPC_MSG__STRUCT_HPP_
