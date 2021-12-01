// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from march_shared_msgs:msg/MpcArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "march_shared_msgs/msg/detail/mpc_array__struct.hpp"
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

void MpcArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) march_shared_msgs::msg::MpcArray(_init);
}

void MpcArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<march_shared_msgs::msg::MpcArray *>(message_memory);
  typed_message->~MpcArray();
}

size_t size_function__MpcArray__array(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MpcArray__array(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__MpcArray__array(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__MpcArray__array(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MpcArray_message_member_array[1] = {
  {
    "array",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::msg::MpcArray, array),  // bytes offset in struct
    nullptr,  // default value
    size_function__MpcArray__array,  // size() function pointer
    get_const_function__MpcArray__array,  // get_const(index) function pointer
    get_function__MpcArray__array,  // get(index) function pointer
    resize_function__MpcArray__array  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MpcArray_message_members = {
  "march_shared_msgs::msg",  // message namespace
  "MpcArray",  // message name
  1,  // number of fields
  sizeof(march_shared_msgs::msg::MpcArray),
  MpcArray_message_member_array,  // message members
  MpcArray_init_function,  // function to initialize message memory (memory has to be allocated)
  MpcArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MpcArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MpcArray_message_members,
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
get_message_type_support_handle<march_shared_msgs::msg::MpcArray>()
{
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::MpcArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, march_shared_msgs, msg, MpcArray)() {
  return &::march_shared_msgs::msg::rosidl_typesupport_introspection_cpp::MpcArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
