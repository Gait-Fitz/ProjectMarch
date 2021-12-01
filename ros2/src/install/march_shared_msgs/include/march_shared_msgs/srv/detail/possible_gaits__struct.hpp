// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:srv/PossibleGaits.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__POSSIBLE_GAITS__STRUCT_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__POSSIBLE_GAITS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__PossibleGaits_Request __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__PossibleGaits_Request __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PossibleGaits_Request_
{
  using Type = PossibleGaits_Request_<ContainerAllocator>;

  explicit PossibleGaits_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit PossibleGaits_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__PossibleGaits_Request
    std::shared_ptr<march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__PossibleGaits_Request
    std::shared_ptr<march_shared_msgs::srv::PossibleGaits_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PossibleGaits_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const PossibleGaits_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PossibleGaits_Request_

// alias to use template instance with default allocator
using PossibleGaits_Request =
  march_shared_msgs::srv::PossibleGaits_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__PossibleGaits_Response __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__PossibleGaits_Response __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PossibleGaits_Response_
{
  using Type = PossibleGaits_Response_<ContainerAllocator>;

  explicit PossibleGaits_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit PossibleGaits_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _gaits_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _gaits_type gaits;

  // setters for named parameter idiom
  Type & set__gaits(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->gaits = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__PossibleGaits_Response
    std::shared_ptr<march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__PossibleGaits_Response
    std::shared_ptr<march_shared_msgs::srv::PossibleGaits_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PossibleGaits_Response_ & other) const
  {
    if (this->gaits != other.gaits) {
      return false;
    }
    return true;
  }
  bool operator!=(const PossibleGaits_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PossibleGaits_Response_

// alias to use template instance with default allocator
using PossibleGaits_Response =
  march_shared_msgs::srv::PossibleGaits_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs

namespace march_shared_msgs
{

namespace srv
{

struct PossibleGaits
{
  using Request = march_shared_msgs::srv::PossibleGaits_Request;
  using Response = march_shared_msgs::srv::PossibleGaits_Response;
};

}  // namespace srv

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__POSSIBLE_GAITS__STRUCT_HPP_
