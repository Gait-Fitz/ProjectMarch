// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryGoal.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'trajectory'
#include "trajectory_msgs/msg/detail/joint_trajectory__struct.hpp"
// Member 'path_tolerance'
// Member 'goal_tolerance'
#include "control_msgs/msg/detail/joint_tolerance__struct.hpp"
// Member 'goal_time_tolerance'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryGoal __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryGoal __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FollowJointTrajectoryGoal_
{
  using Type = FollowJointTrajectoryGoal_<ContainerAllocator>;

  explicit FollowJointTrajectoryGoal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : trajectory(_init),
    goal_time_tolerance(_init)
  {
    (void)_init;
  }

  explicit FollowJointTrajectoryGoal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : trajectory(_alloc, _init),
    goal_time_tolerance(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _trajectory_type =
    trajectory_msgs::msg::JointTrajectory_<ContainerAllocator>;
  _trajectory_type trajectory;
  using _path_tolerance_type =
    std::vector<control_msgs::msg::JointTolerance_<ContainerAllocator>, typename ContainerAllocator::template rebind<control_msgs::msg::JointTolerance_<ContainerAllocator>>::other>;
  _path_tolerance_type path_tolerance;
  using _goal_tolerance_type =
    std::vector<control_msgs::msg::JointTolerance_<ContainerAllocator>, typename ContainerAllocator::template rebind<control_msgs::msg::JointTolerance_<ContainerAllocator>>::other>;
  _goal_tolerance_type goal_tolerance;
  using _goal_time_tolerance_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _goal_time_tolerance_type goal_time_tolerance;

  // setters for named parameter idiom
  Type & set__trajectory(
    const trajectory_msgs::msg::JointTrajectory_<ContainerAllocator> & _arg)
  {
    this->trajectory = _arg;
    return *this;
  }
  Type & set__path_tolerance(
    const std::vector<control_msgs::msg::JointTolerance_<ContainerAllocator>, typename ContainerAllocator::template rebind<control_msgs::msg::JointTolerance_<ContainerAllocator>>::other> & _arg)
  {
    this->path_tolerance = _arg;
    return *this;
  }
  Type & set__goal_tolerance(
    const std::vector<control_msgs::msg::JointTolerance_<ContainerAllocator>, typename ContainerAllocator::template rebind<control_msgs::msg::JointTolerance_<ContainerAllocator>>::other> & _arg)
  {
    this->goal_tolerance = _arg;
    return *this;
  }
  Type & set__goal_time_tolerance(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->goal_time_tolerance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryGoal
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryGoal
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowJointTrajectoryGoal_ & other) const
  {
    if (this->trajectory != other.trajectory) {
      return false;
    }
    if (this->path_tolerance != other.path_tolerance) {
      return false;
    }
    if (this->goal_tolerance != other.goal_tolerance) {
      return false;
    }
    if (this->goal_time_tolerance != other.goal_time_tolerance) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowJointTrajectoryGoal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowJointTrajectoryGoal_

// alias to use template instance with default allocator
using FollowJointTrajectoryGoal =
  march_shared_msgs::msg::FollowJointTrajectoryGoal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_GOAL__STRUCT_HPP_
