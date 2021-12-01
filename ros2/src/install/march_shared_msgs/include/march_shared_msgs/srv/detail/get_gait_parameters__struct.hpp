// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:srv/GetGaitParameters.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__STRUCT_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__GetGaitParameters_Request __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__GetGaitParameters_Request __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetGaitParameters_Request_
{
  using Type = GetGaitParameters_Request_<ContainerAllocator>;

  explicit GetGaitParameters_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->realsense_category = 0;
      this->camera_to_use = 0;
      this->subgait_name = "";
    }
  }

  explicit GetGaitParameters_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : subgait_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->realsense_category = 0;
      this->camera_to_use = 0;
      this->subgait_name = "";
    }
  }

  // field types and members
  using _realsense_category_type =
    uint8_t;
  _realsense_category_type realsense_category;
  using _camera_to_use_type =
    uint8_t;
  _camera_to_use_type camera_to_use;
  using _subgait_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _subgait_name_type subgait_name;

  // setters for named parameter idiom
  Type & set__realsense_category(
    const uint8_t & _arg)
  {
    this->realsense_category = _arg;
    return *this;
  }
  Type & set__camera_to_use(
    const uint8_t & _arg)
  {
    this->camera_to_use = _arg;
    return *this;
  }
  Type & set__subgait_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->subgait_name = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STAIRS_UP =
    0u;
  static constexpr uint8_t STAIRS_DOWN =
    1u;
  static constexpr uint8_t RAMP_UP =
    2u;
  static constexpr uint8_t RAMP_DOWN =
    3u;
  static constexpr uint8_t SIT =
    4u;
  static constexpr uint8_t CURB_UP =
    5u;
  static constexpr uint8_t CURB_DOWN =
    6u;
  static constexpr uint8_t CAMERA_FRONT =
    0u;
  static constexpr uint8_t CAMERA_BACK =
    1u;
  static constexpr uint8_t TEST_CLOUD =
    2u;

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__GetGaitParameters_Request
    std::shared_ptr<march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__GetGaitParameters_Request
    std::shared_ptr<march_shared_msgs::srv::GetGaitParameters_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetGaitParameters_Request_ & other) const
  {
    if (this->realsense_category != other.realsense_category) {
      return false;
    }
    if (this->camera_to_use != other.camera_to_use) {
      return false;
    }
    if (this->subgait_name != other.subgait_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetGaitParameters_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetGaitParameters_Request_

// alias to use template instance with default allocator
using GetGaitParameters_Request =
  march_shared_msgs::srv::GetGaitParameters_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::STAIRS_UP;
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::STAIRS_DOWN;
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::RAMP_UP;
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::RAMP_DOWN;
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::SIT;
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::CURB_UP;
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::CURB_DOWN;
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::CAMERA_FRONT;
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::CAMERA_BACK;
template<typename ContainerAllocator>
constexpr uint8_t GetGaitParameters_Request_<ContainerAllocator>::TEST_CLOUD;

}  // namespace srv

}  // namespace march_shared_msgs


// Include directives for member types
// Member 'gait_parameters'
#include "march_shared_msgs/msg/detail/gait_parameters__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__GetGaitParameters_Response __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__GetGaitParameters_Response __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetGaitParameters_Response_
{
  using Type = GetGaitParameters_Response_<ContainerAllocator>;

  explicit GetGaitParameters_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : gait_parameters(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
    }
  }

  explicit GetGaitParameters_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_message(_alloc),
    gait_parameters(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _error_message_type error_message;
  using _gait_parameters_type =
    march_shared_msgs::msg::GaitParameters_<ContainerAllocator>;
  _gait_parameters_type gait_parameters;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }
  Type & set__gait_parameters(
    const march_shared_msgs::msg::GaitParameters_<ContainerAllocator> & _arg)
  {
    this->gait_parameters = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__GetGaitParameters_Response
    std::shared_ptr<march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__GetGaitParameters_Response
    std::shared_ptr<march_shared_msgs::srv::GetGaitParameters_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetGaitParameters_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    if (this->gait_parameters != other.gait_parameters) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetGaitParameters_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetGaitParameters_Response_

// alias to use template instance with default allocator
using GetGaitParameters_Response =
  march_shared_msgs::srv::GetGaitParameters_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs

namespace march_shared_msgs
{

namespace srv
{

struct GetGaitParameters
{
  using Request = march_shared_msgs::srv::GetGaitParameters_Request;
  using Response = march_shared_msgs::srv::GetGaitParameters_Response;
};

}  // namespace srv

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_GAIT_PARAMETERS__STRUCT_HPP_
