// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:srv/PublishTestDataset.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__PUBLISH_TEST_DATASET__STRUCT_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__PUBLISH_TEST_DATASET__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__PublishTestDataset_Request __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__PublishTestDataset_Request __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PublishTestDataset_Request_
{
  using Type = PublishTestDataset_Request_<ContainerAllocator>;

  explicit PublishTestDataset_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->selected_mode = 0;
      this->pointcloud_file_name = "";
      this->save_camera_back = false;
    }
  }

  explicit PublishTestDataset_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pointcloud_file_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->selected_mode = 0;
      this->pointcloud_file_name = "";
      this->save_camera_back = false;
    }
  }

  // field types and members
  using _selected_mode_type =
    uint8_t;
  _selected_mode_type selected_mode;
  using _pointcloud_file_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _pointcloud_file_name_type pointcloud_file_name;
  using _save_camera_back_type =
    bool;
  _save_camera_back_type save_camera_back;

  // setters for named parameter idiom
  Type & set__selected_mode(
    const uint8_t & _arg)
  {
    this->selected_mode = _arg;
    return *this;
  }
  Type & set__pointcloud_file_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->pointcloud_file_name = _arg;
    return *this;
  }
  Type & set__save_camera_back(
    const bool & _arg)
  {
    this->save_camera_back = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NEXT =
    0u;
  static constexpr uint8_t STOP =
    1u;
  static constexpr uint8_t CUSTOM =
    2u;
  static constexpr uint8_t SAVE =
    3u;

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__PublishTestDataset_Request
    std::shared_ptr<march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__PublishTestDataset_Request
    std::shared_ptr<march_shared_msgs::srv::PublishTestDataset_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PublishTestDataset_Request_ & other) const
  {
    if (this->selected_mode != other.selected_mode) {
      return false;
    }
    if (this->pointcloud_file_name != other.pointcloud_file_name) {
      return false;
    }
    if (this->save_camera_back != other.save_camera_back) {
      return false;
    }
    return true;
  }
  bool operator!=(const PublishTestDataset_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PublishTestDataset_Request_

// alias to use template instance with default allocator
using PublishTestDataset_Request =
  march_shared_msgs::srv::PublishTestDataset_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t PublishTestDataset_Request_<ContainerAllocator>::NEXT;
template<typename ContainerAllocator>
constexpr uint8_t PublishTestDataset_Request_<ContainerAllocator>::STOP;
template<typename ContainerAllocator>
constexpr uint8_t PublishTestDataset_Request_<ContainerAllocator>::CUSTOM;
template<typename ContainerAllocator>
constexpr uint8_t PublishTestDataset_Request_<ContainerAllocator>::SAVE;

}  // namespace srv

}  // namespace march_shared_msgs


#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__PublishTestDataset_Response __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__PublishTestDataset_Response __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PublishTestDataset_Response_
{
  using Type = PublishTestDataset_Response_<ContainerAllocator>;

  explicit PublishTestDataset_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit PublishTestDataset_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__PublishTestDataset_Response
    std::shared_ptr<march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__PublishTestDataset_Response
    std::shared_ptr<march_shared_msgs::srv::PublishTestDataset_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PublishTestDataset_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PublishTestDataset_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PublishTestDataset_Response_

// alias to use template instance with default allocator
using PublishTestDataset_Response =
  march_shared_msgs::srv::PublishTestDataset_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs

namespace march_shared_msgs
{

namespace srv
{

struct PublishTestDataset
{
  using Request = march_shared_msgs::srv::PublishTestDataset_Request;
  using Response = march_shared_msgs::srv::PublishTestDataset_Response;
};

}  // namespace srv

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__PUBLISH_TEST_DATASET__STRUCT_HPP_
