// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from march_shared_msgs:msg/MotorControllerState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "march_shared_msgs/msg/detail/motor_controller_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace march_shared_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorControllerState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) march_shared_msgs::msg::MotorControllerState(_init);
}

void MotorControllerState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<march_shared_msgs::msg::MotorControllerState *>(message_memory);
  typed_message->~MotorControllerState();
}

size_t size_function__MotorControllerState__joint_names(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__joint_names(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__joint_names(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__joint_names(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__error_status(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__error_status(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__error_status(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__error_status(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__operational_state(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__operational_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__operational_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__operational_state(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__motor_current(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__motor_current(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__motor_current(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__motor_current(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__motor_voltage(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__motor_voltage(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__motor_voltage(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__motor_voltage(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__temperature(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__temperature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__temperature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__temperature(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__absolute_position_iu(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__absolute_position_iu(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__absolute_position_iu(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__absolute_position_iu(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__incremental_position_iu(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__incremental_position_iu(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__incremental_position_iu(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__incremental_position_iu(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__absolute_velocity_iu(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__absolute_velocity_iu(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__absolute_velocity_iu(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__absolute_velocity_iu(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__incremental_velocity_iu(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__incremental_velocity_iu(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__incremental_velocity_iu(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__incremental_velocity_iu(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__absolute_position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__absolute_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__absolute_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__absolute_position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__incremental_position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__incremental_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__incremental_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__incremental_position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__absolute_velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__absolute_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__absolute_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__absolute_velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorControllerState__incremental_velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorControllerState__incremental_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorControllerState__incremental_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MotorControllerState__incremental_velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorControllerState_message_member_array[15] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joint_names",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, joint_names),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__joint_names,  // size() function pointer
    get_const_function__MotorControllerState__joint_names,  // get_const(index) function pointer
    get_function__MotorControllerState__joint_names,  // get(index) function pointer
    resize_function__MotorControllerState__joint_names  // resize(index) function pointer
  },
  {
    "error_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, error_status),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__error_status,  // size() function pointer
    get_const_function__MotorControllerState__error_status,  // get_const(index) function pointer
    get_function__MotorControllerState__error_status,  // get(index) function pointer
    resize_function__MotorControllerState__error_status  // resize(index) function pointer
  },
  {
    "operational_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, operational_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__operational_state,  // size() function pointer
    get_const_function__MotorControllerState__operational_state,  // get_const(index) function pointer
    get_function__MotorControllerState__operational_state,  // get(index) function pointer
    resize_function__MotorControllerState__operational_state  // resize(index) function pointer
  },
  {
    "motor_current",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, motor_current),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__motor_current,  // size() function pointer
    get_const_function__MotorControllerState__motor_current,  // get_const(index) function pointer
    get_function__MotorControllerState__motor_current,  // get(index) function pointer
    resize_function__MotorControllerState__motor_current  // resize(index) function pointer
  },
  {
    "motor_voltage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, motor_voltage),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__motor_voltage,  // size() function pointer
    get_const_function__MotorControllerState__motor_voltage,  // get_const(index) function pointer
    get_function__MotorControllerState__motor_voltage,  // get(index) function pointer
    resize_function__MotorControllerState__motor_voltage  // resize(index) function pointer
  },
  {
    "temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, temperature),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__temperature,  // size() function pointer
    get_const_function__MotorControllerState__temperature,  // get_const(index) function pointer
    get_function__MotorControllerState__temperature,  // get(index) function pointer
    resize_function__MotorControllerState__temperature  // resize(index) function pointer
  },
  {
    "absolute_position_iu",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, absolute_position_iu),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__absolute_position_iu,  // size() function pointer
    get_const_function__MotorControllerState__absolute_position_iu,  // get_const(index) function pointer
    get_function__MotorControllerState__absolute_position_iu,  // get(index) function pointer
    resize_function__MotorControllerState__absolute_position_iu  // resize(index) function pointer
  },
  {
    "incremental_position_iu",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, incremental_position_iu),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__incremental_position_iu,  // size() function pointer
    get_const_function__MotorControllerState__incremental_position_iu,  // get_const(index) function pointer
    get_function__MotorControllerState__incremental_position_iu,  // get(index) function pointer
    resize_function__MotorControllerState__incremental_position_iu  // resize(index) function pointer
  },
  {
    "absolute_velocity_iu",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, absolute_velocity_iu),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__absolute_velocity_iu,  // size() function pointer
    get_const_function__MotorControllerState__absolute_velocity_iu,  // get_const(index) function pointer
    get_function__MotorControllerState__absolute_velocity_iu,  // get(index) function pointer
    resize_function__MotorControllerState__absolute_velocity_iu  // resize(index) function pointer
  },
  {
    "incremental_velocity_iu",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, incremental_velocity_iu),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__incremental_velocity_iu,  // size() function pointer
    get_const_function__MotorControllerState__incremental_velocity_iu,  // get_const(index) function pointer
    get_function__MotorControllerState__incremental_velocity_iu,  // get(index) function pointer
    resize_function__MotorControllerState__incremental_velocity_iu  // resize(index) function pointer
  },
  {
    "absolute_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, absolute_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__absolute_position,  // size() function pointer
    get_const_function__MotorControllerState__absolute_position,  // get_const(index) function pointer
    get_function__MotorControllerState__absolute_position,  // get(index) function pointer
    resize_function__MotorControllerState__absolute_position  // resize(index) function pointer
  },
  {
    "incremental_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, incremental_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__incremental_position,  // size() function pointer
    get_const_function__MotorControllerState__incremental_position,  // get_const(index) function pointer
    get_function__MotorControllerState__incremental_position,  // get(index) function pointer
    resize_function__MotorControllerState__incremental_position  // resize(index) function pointer
  },
  {
    "absolute_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, absolute_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__absolute_velocity,  // size() function pointer
    get_const_function__MotorControllerState__absolute_velocity,  // get_const(index) function pointer
    get_function__MotorControllerState__absolute_velocity,  // get(index) function pointer
    resize_function__MotorControllerState__absolute_velocity  // resize(index) function pointer
  },
  {
    "incremental_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MotorControllerState, incremental_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorControllerState__incremental_velocity,  // size() function pointer
    get_const_function__MotorControllerState__incremental_velocity,  // get_const(index) function pointer
    get_function__MotorControllerState__incremental_velocity,  // get(index) function pointer
    resize_function__MotorControllerState__incremental_velocity  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorControllerState_message_members = {
  "march_shared_msgs::msg",  // message namespace
  "MotorControllerState",  // message name
  15,  // number of fields
  sizeof(march_shared_msgs::msg::MotorControllerState),
  MotorControllerState_message_member_array,  // message members
  MotorControllerState_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorControllerState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorControllerState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorControllerState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace march_shared_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<march_shared_msgs::msg::MotorControllerState>()
{
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::MotorControllerState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, march_shared_msgs, msg, MotorControllerState)() {
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::MotorControllerState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
