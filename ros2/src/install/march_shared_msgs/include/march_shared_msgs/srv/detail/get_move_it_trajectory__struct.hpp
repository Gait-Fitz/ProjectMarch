// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:srv/GetMoveItTrajectory.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__STRUCT_HPP_
#define MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'swing_leg_target_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'stance_leg_target'
#include "sensor_msgs/msg/detail/joint_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__GetMoveItTrajectory_Request __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__GetMoveItTrajectory_Request __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMoveItTrajectory_Request_
{
  using Type = GetMoveItTrajectory_Request_<ContainerAllocator>;

  explicit GetMoveItTrajectory_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : swing_leg_target_pose(_init),
    stance_leg_target(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->swing_leg = "";
    }
  }

  explicit GetMoveItTrajectory_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : swing_leg(_alloc),
    swing_leg_target_pose(_alloc, _init),
    stance_leg_target(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->swing_leg = "";
    }
  }

  // field types and members
  using _swing_leg_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _swing_leg_type swing_leg;
  using _swing_leg_target_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _swing_leg_target_pose_type swing_leg_target_pose;
  using _stance_leg_target_type =
    sensor_msgs::msg::JointState_<ContainerAllocator>;
  _stance_leg_target_type stance_leg_target;

  // setters for named parameter idiom
  Type & set__swing_leg(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->swing_leg = _arg;
    return *this;
  }
  Type & set__swing_leg_target_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->swing_leg_target_pose = _arg;
    return *this;
  }
  Type & set__stance_leg_target(
    const sensor_msgs::msg::JointState_<ContainerAllocator> & _arg)
  {
    this->stance_leg_target = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__GetMoveItTrajectory_Request
    std::shared_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__GetMoveItTrajectory_Request
    std::shared_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMoveItTrajectory_Request_ & other) const
  {
    if (this->swing_leg != other.swing_leg) {
      return false;
    }
    if (this->swing_leg_target_pose != other.swing_leg_target_pose) {
      return false;
    }
    if (this->stance_leg_target != other.stance_leg_target) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMoveItTrajectory_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMoveItTrajectory_Request_

// alias to use template instance with default allocator
using GetMoveItTrajectory_Request =
  march_shared_msgs::srv::GetMoveItTrajectory_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs


// Include directives for member types
// Member 'trajectory'
#include "trajectory_msgs/msg/detail/joint_trajectory__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__srv__GetMoveItTrajectory_Response __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__srv__GetMoveItTrajectory_Response __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMoveItTrajectory_Response_
{
  using Type = GetMoveItTrajectory_Response_<ContainerAllocator>;

  explicit GetMoveItTrajectory_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : trajectory(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit GetMoveItTrajectory_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : trajectory(_alloc, _init)
  {
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
  using _trajectory_type =
    trajectory_msgs::msg::JointTrajectory_<ContainerAllocator>;
  _trajectory_type trajectory;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__trajectory(
    const trajectory_msgs::msg::JointTrajectory_<ContainerAllocator> & _arg)
  {
    this->trajectory = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__srv__GetMoveItTrajectory_Response
    std::shared_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__srv__GetMoveItTrajectory_Response
    std::shared_ptr<march_shared_msgs::srv::GetMoveItTrajectory_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMoveItTrajectory_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->trajectory != other.trajectory) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMoveItTrajectory_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMoveItTrajectory_Response_

// alias to use template instance with default allocator
using GetMoveItTrajectory_Response =
  march_shared_msgs::srv::GetMoveItTrajectory_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace march_shared_msgs

namespace march_shared_msgs
{

namespace srv
{

struct GetMoveItTrajectory
{
  using Request = march_shared_msgs::srv::GetMoveItTrajectory_Request;
  using Response = march_shared_msgs::srv::GetMoveItTrajectory_Response;
};

}  // namespace srv

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__SRV__DETAIL__GET_MOVE_IT_TRAJECTORY__STRUCT_HPP_
