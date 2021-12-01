// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:srv/ContainsGait.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__CONTAINS_GAIT__STRUCT_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__CONTAINS_GAIT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__ContainsGait_Request __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__ContainsGait_Request __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ContainsGait_Request_
{
  using Type = ContainsGait_Request_<ContainerAllocator>;

  explicit ContainsGait_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gait = "";
    }
  }

  explicit ContainsGait_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : gait(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gait = "";
    }
  }

  // field types and members
  using _gait_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _gait_type gait;
  using _subgaits_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _subgaits_type subgaits;

  // setters for named parameter idiom
  Type & set__gait(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->gait = _arg;
    return *this;
  }
  Type & set__subgaits(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->subgaits = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__ContainsGait_Request
    std::shared_ptr<march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__ContainsGait_Request
    std::shared_ptr<march_shared_msgs::srv::ContainsGait_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContainsGait_Request_ & other) const
  {
    if (this->gait != other.gait) {
      return false;
    }
    if (this->subgaits != other.subgaits) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContainsGait_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContainsGait_Request_

// alias to use template instance with default allocator
using ContainsGait_Request =
  march_shared_msgs::srv::ContainsGait_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__ContainsGait_Response __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__ContainsGait_Response __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ContainsGait_Response_
{
  using Type = ContainsGait_Response_<ContainerAllocator>;

  explicit ContainsGait_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->contains = false;
    }
  }

  explicit ContainsGait_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->contains = false;
    }
  }

  // field types and members
  using _contains_type =
    bool;
  _contains_type contains;

  // setters for named parameter idiom
  Type & set__contains(
    const bool & _arg)
  {
    this->contains = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__ContainsGait_Response
    std::shared_ptr<march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__ContainsGait_Response
    std::shared_ptr<march_shared_msgs::srv::ContainsGait_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContainsGait_Response_ & other) const
  {
    if (this->contains != other.contains) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContainsGait_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContainsGait_Response_

// alias to use template instance with default allocator
using ContainsGait_Response =
  march_shared_msgs::srv::ContainsGait_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs

namespace march_shared_msgs
{

namespace srv
{

struct ContainsGait
{
  using Request = march_shared_msgs::srv::ContainsGait_Request;
  using Response = march_shared_msgs::srv::ContainsGait_Response;
};

}  // namespace srv

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__CONTAINS_GAIT__STRUCT_HPP_
