// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/GaitInstructionResponse.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION_RESPONSE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION_RESPONSE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__GaitInstructionResponse __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__GaitInstructionResponse __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GaitInstructionResponse_
{
  using Type = GaitInstructionResponse_<ContainerAllocator>;

  explicit GaitInstructionResponse_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
    }
  }

  explicit GaitInstructionResponse_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
    }
  }

  // field types and members
  using _result_type =
    int8_t;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const int8_t & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t GAIT_REJECTED =
    -1;
  static constexpr int8_t GAIT_ACCEPTED =
    0;
  static constexpr int8_t GAIT_FINISHED =
    1;

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__GaitInstructionResponse
    std::shared_ptr<march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__GaitInstructionResponse
    std::shared_ptr<march_shared_msgs::msg::GaitInstructionResponse_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GaitInstructionResponse_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const GaitInstructionResponse_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GaitInstructionResponse_

// alias to use template instance with default allocator
using GaitInstructionResponse =
  march_shared_msgs::msg::GaitInstructionResponse_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t GaitInstructionResponse_<ContainerAllocator>::GAIT_REJECTED;
template<typename ContainerAllocator>
constexpr int8_t GaitInstructionResponse_<ContainerAllocator>::GAIT_ACCEPTED;
template<typename ContainerAllocator>
constexpr int8_t GaitInstructionResponse_<ContainerAllocator>::GAIT_FINISHED;

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_INSTRUCTION_RESPONSE__STRUCT_HPP_
