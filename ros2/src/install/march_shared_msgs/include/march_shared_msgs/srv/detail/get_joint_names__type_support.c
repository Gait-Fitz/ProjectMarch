// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:srv/GetJointNames.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/srv/detail/get_joint_names__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/srv/detail/get_joint_names__functions.h"
#include "march_shared_msgs/srv/detail/get_joint_names__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__srv__GetJointNames_Request__init(message_memory);
}

void GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_fini_function(void * message_memory)
{
  march_shared_msgs__srv__GetJointNames_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__srv__GetJointNames_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_message_members = {
  "march_shared_msgs__srv",  // message namespace
  "GetJointNames_Request",  // message name
  1,  // number of fields
  sizeof(march_shared_msgs__srv__GetJointNames_Request),
  GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_message_member_array,  // message members
  GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_message_type_support_handle = {
  0,
  &GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, GetJointNames_Request)() {
  if (!GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_message_type_support_handle.typesupport_identifier) {
    GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetJointNames_Request__rosidl_typesupport_introspection_c__GetJointNames_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "march_shared_msgs/srv/detail/get_joint_names__rosidl_typesupport_introspection_c.h"
// already included above
// #include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "march_shared_msgs/srv/detail/get_joint_names__functions.h"
// already included above
// #include "march_shared_msgs/srv/detail/get_joint_names__struct.h"


// Include directives for member types
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__srv__GetJointNames_Response__init(message_memory);
}

void GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_fini_function(void * message_memory)
{
  march_shared_msgs__srv__GetJointNames_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_message_member_array[1] = {
  {
    "joint_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__srv__GetJointNames_Response, joint_names),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_message_members = {
  "march_shared_msgs__srv",  // message namespace
  "GetJointNames_Response",  // message name
  1,  // number of fields
  sizeof(march_shared_msgs__srv__GetJointNames_Response),
  GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_message_member_array,  // message members
  GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_message_type_support_handle = {
  0,
  &GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, GetJointNames_Response)() {
  if (!GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_message_type_support_handle.typesupport_identifier) {
    GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetJointNames_Response__rosidl_typesupport_introspection_c__GetJointNames_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "march_shared_msgs/srv/detail/get_joint_names__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers march_shared_msgs__srv__detail__get_joint_names__rosidl_typesupport_introspection_c__GetJointNames_service_members = {
  "march_shared_msgs__srv",  // service namespace
  "GetJointNames",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // march_shared_msgs__srv__detail__get_joint_names__rosidl_typesupport_introspection_c__GetJointNames_Request_message_type_support_handle,
  NULL  // response message
  // march_shared_msgs__srv__detail__get_joint_names__rosidl_typesupport_introspection_c__GetJointNames_Response_message_type_support_handle
};

static rosidl_service_type_support_t march_shared_msgs__srv__detail__get_joint_names__rosidl_typesupport_introspection_c__GetJointNames_service_type_support_handle = {
  0,
  &march_shared_msgs__srv__detail__get_joint_names__rosidl_typesupport_introspection_c__GetJointNames_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, GetJointNames_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, GetJointNames_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, GetJointNames)() {
  if (!march_shared_msgs__srv__detail__get_joint_names__rosidl_typesupport_introspection_c__GetJointNames_service_type_support_handle.typesupport_identifier) {
    march_shared_msgs__srv__detail__get_joint_names__rosidl_typesupport_introspection_c__GetJointNames_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)march_shared_msgs__srv__detail__get_joint_names__rosidl_typesupport_introspection_c__GetJointNames_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, GetJointNames_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, GetJointNames_Response)()->data;
  }

  return &march_shared_msgs__srv__detail__get_joint_names__rosidl_typesupport_introspection_c__GetJointNames_service_type_support_handle;
}
