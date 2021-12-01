// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from march_shared_msgs:msg/MpcTuning.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "march_shared_msgs/msg/detail/mpc_tuning__struct.hpp"
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

void MpcTuning_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) march_shared_msgs::msg::MpcTuning(_init);
}

void MpcTuning_fini_function(void * message_memory)
{
  auto typed_message = static_cast<march_shared_msgs::msg::MpcTuning *>(message_memory);
  typed_message->~MpcTuning();
}

size_t size_function__MpcTuning__q_weights(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MpcTuning__q_weights(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MpcTuning__q_weights(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MpcTuning__q_weights(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MpcTuning__r_weights(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MpcTuning__r_weights(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MpcTuning__r_weights(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MpcTuning__r_weights(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MpcTuning_message_member_array[3] = {
  {
    "q_weights",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MpcTuning, q_weights),  // bytes offset in struct
    nullptr,  // default value
    size_function__MpcTuning__q_weights,  // size() function pointer
    get_const_function__MpcTuning__q_weights,  // get_const(index) function pointer
    get_function__MpcTuning__q_weights,  // get(index) function pointer
    resize_function__MpcTuning__q_weights  // resize(index) function pointer
  },
  {
    "r_weights",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MpcTuning, r_weights),  // bytes offset in struct
    nullptr,  // default value
    size_function__MpcTuning__r_weights,  // size() function pointer
    get_const_function__MpcTuning__r_weights,  // get_const(index) function pointer
    get_function__MpcTuning__r_weights,  // get(index) function pointer
    resize_function__MpcTuning__r_weights  // resize(index) function pointer
  },
  {
    "horizon",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MpcTuning, horizon),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MpcTuning_message_members = {
  "march_shared_msgs::msg",  // message namespace
  "MpcTuning",  // message name
  3,  // number of fields
  sizeof(march_shared_msgs::msg::MpcTuning),
  MpcTuning_message_member_array,  // message members
  MpcTuning_init_function,  // function to initialize message memory (memory has to be allocated)
  MpcTuning_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MpcTuning_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MpcTuning_message_members,
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
get_message_type_support_handle<march_shared_msgs::msg::MpcTuning>()
{
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::MpcTuning_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, march_shared_msgs, msg, MpcTuning)() {
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::MpcTuning_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
