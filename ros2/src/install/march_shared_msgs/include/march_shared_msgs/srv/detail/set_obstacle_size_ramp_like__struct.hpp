// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:srv/SetObstacleSizeRampLike.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_RAMP_LIKE__STRUCT_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_RAMP_LIKE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__SetObstacleSizeRampLike_Request __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__SetObstacleSizeRampLike_Request __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetObstacleSizeRampLike_Request_
{
  using Type = SetObstacleSizeRampLike_Request_<ContainerAllocator>;

  explicit SetObstacleSizeRampLike_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->obstacle_name = "";
      this->new_slope_up = 0.0;
      this->new_slope_down = 0.0;
      this->new_ramp_up_length = 0.0;
    }
  }

  explicit SetObstacleSizeRampLike_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : obstacle_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->obstacle_name = "";
      this->new_slope_up = 0.0;
      this->new_slope_down = 0.0;
      this->new_ramp_up_length = 0.0;
    }
  }

  // field types and members
  using _obstacle_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _obstacle_name_type obstacle_name;
  using _new_slope_up_type =
    double;
  _new_slope_up_type new_slope_up;
  using _new_slope_down_type =
    double;
  _new_slope_down_type new_slope_down;
  using _new_ramp_up_length_type =
    double;
  _new_ramp_up_length_type new_ramp_up_length;

  // setters for named parameter idiom
  Type & set__obstacle_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->obstacle_name = _arg;
    return *this;
  }
  Type & set__new_slope_up(
    const double & _arg)
  {
    this->new_slope_up = _arg;
    return *this;
  }
  Type & set__new_slope_down(
    const double & _arg)
  {
    this->new_slope_down = _arg;
    return *this;
  }
  Type & set__new_ramp_up_length(
    const double & _arg)
  {
    this->new_ramp_up_length = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__SetObstacleSizeRampLike_Request
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__SetObstacleSizeRampLike_Request
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetObstacleSizeRampLike_Request_ & other) const
  {
    if (this->obstacle_name != other.obstacle_name) {
      return false;
    }
    if (this->new_slope_up != other.new_slope_up) {
      return false;
    }
    if (this->new_slope_down != other.new_slope_down) {
      return false;
    }
    if (this->new_ramp_up_length != other.new_ramp_up_length) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetObstacleSizeRampLike_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetObstacleSizeRampLike_Request_

// alias to use template instance with default allocator
using SetObstacleSizeRampLike_Request =
  march_shared_msgs::srv::SetObstacleSizeRampLike_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__SetObstacleSizeRampLike_Response __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__SetObstacleSizeRampLike_Response __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetObstacleSizeRampLike_Response_
{
  using Type = SetObstacleSizeRampLike_Response_<ContainerAllocator>;

  explicit SetObstacleSizeRampLike_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetObstacleSizeRampLike_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__SetObstacleSizeRampLike_Response
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__SetObstacleSizeRampLike_Response
    std::shared_ptr<march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetObstacleSizeRampLike_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetObstacleSizeRampLike_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetObstacleSizeRampLike_Response_

// alias to use template instance with default allocator
using SetObstacleSizeRampLike_Response =
  march_shared_msgs::srv::SetObstacleSizeRampLike_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs

namespace march_shared_msgs
{

namespace srv
{

struct SetObstacleSizeRampLike
{
  using Request = march_shared_msgs::srv::SetObstacleSizeRampLike_Request;
  using Response = march_shared_msgs::srv::SetObstacleSizeRampLike_Response;
};

}  // namespace srv

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__SET_OBSTACLE_SIZE_RAMP_LIKE__STRUCT_HPP_
