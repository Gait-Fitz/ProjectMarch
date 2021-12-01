// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from march_shared_msgs:msg/FollowJointTrajectoryActionGoal.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_GOAL__STRUCT_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_GOAL__STRUCT_HPP_

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
// Member 'goal_id'
#include "actionlib_msgs/msg/detail/goal_id__struct.hpp"
// Member 'goal'
#include "march_shared_msgs/msg/detail/follow_joint_trajectory_goal__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryActionGoal __attribute__((deprecated))
#else
# define DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryActionGoal __declspec(deprecated)
#endif

namespace march_shared_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FollowJointTrajectoryActionGoal_
{
  using Type = FollowJointTrajectoryActionGoal_<ContainerAllocator>;

  explicit FollowJointTrajectoryActionGoal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit FollowJointTrajectoryActionGoal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _goal_id_type =
    actionlib_msgs::msg::GoalID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__goal_id(
    const actionlib_msgs::msg::GoalID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const march_shared_msgs::msg::FollowJointTrajectoryGoal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator> *;
  using ConstRawPtr =
    const march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryActionGoal
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__march_shared_msgs__msg__FollowJointTrajectoryActionGoal
    std::shared_ptr<march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowJointTrajectoryActionGoal_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowJointTrajectoryActionGoal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowJointTrajectoryActionGoal_

// alias to use template instance with default allocator
using FollowJointTrajectoryActionGoal =
  march_shared_msgs::msg::FollowJointTrajectoryActionGoal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__FOLLOW_JOINT_TRAJECTORY_ACTION_GOAL__STRUCT_HPP_
