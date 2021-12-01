// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:srv/SetObstacleSizeBlockLike.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_BLOCK_LIKE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_BLOCK_LIKE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__SetObstacleSizeBlockLike_Request __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__SetObstacleSizeBlockLike_Request __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetObstacleSizeBlockLike_Request_
{
  using Type = SetObstacleSizeBlockLike_Request_<ContainerAllocator>;

  explicit SetObstacleSizeBlockLike_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->obstacle_name = "";
      this->new_height = 0.0;
      this->new_width = 0.0;
      this->new_length = 0.0;
    }
  }

  explicit SetObstacleSizeBlockLike_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : obstacle_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->obstacle_name = "";
      this->new_height = 0.0;
      this->new_width = 0.0;
      this->new_length = 0.0;
    }
  }

  // field types and members
  using _obstacle_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _obstacle_name_type obstacle_name;
  using _new_height_type =
    double;
  _new_height_type new_height;
  using _new_width_type =
    double;
  _new_width_type new_width;
  using _new_length_type =
    double;
  _new_length_type new_length;

  // setters for named parameter idiom
  Type & set__obstacle_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->obstacle_name = _arg;
    return *this;
  }
  Type & set__new_height(
    const double & _arg)
  {
    this->new_height = _arg;
    return *this;
  }
  Type & set__new_width(
    const double & _arg)
  {
    this->new_width = _arg;
    return *this;
  }
  Type & set__new_length(
    const double & _arg)
  {
    this->new_length = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__SetObstacleSizeBlockLike_Request
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__SetObstacleSizeBlockLike_Request
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetObstacleSizeBlockLike_Request_ & other) const
  {
    if (this->obstacle_name != other.obstacle_name) {
      return false;
    }
    if (this->new_height != other.new_height) {
      return false;
    }
    if (this->new_width != other.new_width) {
      return false;
    }
    if (this->new_length != other.new_length) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetObstacleSizeBlockLike_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetObstacleSizeBlockLike_Request_

// alias to use template instance with default allocator
using SetObstacleSizeBlockLike_Request =
  march_shared_msgs::srv::SetObstacleSizeBlockLike_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__SetObstacleSizeBlockLike_Response __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__SetObstacleSizeBlockLike_Response __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetObstacleSizeBlockLike_Response_
{
  using Type = SetObstacleSizeBlockLike_Response_<ContainerAllocator>;

  explicit SetObstacleSizeBlockLike_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetObstacleSizeBlockLike_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__SetObstacleSizeBlockLike_Response
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__SetObstacleSizeBlockLike_Response
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetObstacleSizeBlockLike_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetObstacleSizeBlockLike_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetObstacleSizeBlockLike_Response_

// alias to use template instance with default allocator
using SetObstacleSizeBlockLike_Response =
  march_shared_msgs::srv::SetObstacleSizeBlockLike_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs

namespace march_shared_msgs
{

namespace srv
{

struct SetObstacleSizeBlockLike
{
  using Request = march_shared_msgs::srv::SetObstacleSizeBlockLike_Request;
  using Response = march_shared_msgs::srv::SetObstacleSizeBlockLike_Response;
};

}  // namespace srv

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_BLOCK_LIKE__STRUCT_HPP_
