// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from march_shared_msgs:srv/SetGaitVersion.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "march_shared_msgs/srv/detail/set_gait_version__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace march_shared_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetGaitVersion_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) march_shared_msgs::srv::SetGaitVersion_Request(_init);
}

void SetGaitVersion_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<march_shared_msgs::srv::SetGaitVersion_Request *>(message_memory);
  typed_message->~SetGaitVersion_Request();
}

size_t size_function__SetGaitVersion_Request__subgaits(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SetGaitVersion_Request__subgaits(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__SetGaitVersion_Request__subgaits(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__SetGaitVersion_Request__subgaits(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SetGaitVersion_Request__versions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SetGaitVersion_Request__versions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__SetGaitVersion_Request__versions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__SetGaitVersion_Request__versions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetGaitVersion_Request_message_member_array[3] = {
  {
    "gait",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::srv::SetGaitVersion_Request, gait),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "subgaits",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::srv::SetGaitVersion_Request, subgaits),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetGaitVersion_Request__subgaits,  // size() function pointer
    get_const_function__SetGaitVersion_Request__subgaits,  // get_const(index) function pointer
    get_function__SetGaitVersion_Request__subgaits,  // get(index) function pointer
    resize_function__SetGaitVersion_Request__subgaits  // resize(index) function pointer
  },
  {
    "versions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::srv::SetGaitVersion_Request, versions),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetGaitVersion_Request__versions,  // size() function pointer
    get_const_function__SetGaitVersion_Request__versions,  // get_const(index) function pointer
    get_function__SetGaitVersion_Request__versions,  // get(index) function pointer
    resize_function__SetGaitVersion_Request__versions  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetGaitVersion_Request_message_members = {
  "march_shared_msgs::srv",  // message namespace
  "SetGaitVersion_Request",  // message name
  3,  // number of fields
  sizeof(march_shared_msgs::srv::SetGaitVersion_Request),
  SetGaitVersion_Request_message_member_array,  // message members
  SetGaitVersion_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SetGaitVersion_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetGaitVersion_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetGaitVersion_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace march_shared_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<march_shared_msgs::srv::SetGaitVersion_Request>()
{
  return &::march_shared_msgs::srv::rosidl_typesupport_introspection_cpp::SetGaitVersion_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, march_shared_msgs, srv, SetGaitVersion_Request)() {
  return &::march_shared_msgs::srv::rosidl_typesupport_introspection_cpp::SetGaitVersion_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "march_shared_msgs/srv/detail/set_gait_version__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace march_shared_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetGaitVersion_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) march_shared_msgs::srv::SetGaitVersion_Response(_init);
}

void SetGaitVersion_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<march_shared_msgs::srv::SetGaitVersion_Response *>(message_memory);
  typed_message->~SetGaitVersion_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetGaitVersion_Response_message_member_array[2] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::srv::SetGaitVersion_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "message",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs::srv::SetGaitVersion_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetGaitVersion_Response_message_members = {
  "march_shared_msgs::srv",  // message namespace
  "SetGaitVersion_Response",  // message name
  2,  // number of fields
  sizeof(march_shared_msgs::srv::SetGaitVersion_Response),
  SetGaitVersion_Response_message_member_array,  // message members
  SetGaitVersion_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SetGaitVersion_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetGaitVersion_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetGaitVersion_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace march_shared_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<march_shared_msgs::srv::SetGaitVersion_Response>()
{
  return &::march_shared_msgs::srv::rosidl_typesupport_introspection_cpp::SetGaitVersion_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, march_shared_msgs, srv, SetGaitVersion_Response)() {
  return &::march_shared_msgs::srv::rosidl_typesupport_introspection_cpp::SetGaitVersion_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "march_shared_msgs/srv/detail/set_gait_version__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace march_shared_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SetGaitVersion_service_members = {
  "march_shared_msgs::srv",  // service namespace
  "SetGaitVersion",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<march_shared_msgs::srv::SetGaitVersion>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t SetGaitVersion_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetGaitVersion_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace march_shared_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<march_shared_msgs::srv::SetGaitVersion>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::march_shared_msgs::srv::rosidl_typesupport_introspection_cpp::SetGaitVersion_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::march_shared_msgs::srv::SetGaitVersion_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::march_shared_msgs::srv::SetGaitVersion_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, march_shared_msgs, srv, SetGaitVersion)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<march_shared_msgs::srv::SetGaitVersion>();
}

#ifdef __cplusplus
}
#endif
