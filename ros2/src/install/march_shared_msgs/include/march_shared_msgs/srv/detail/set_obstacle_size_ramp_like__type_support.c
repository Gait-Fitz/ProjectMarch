// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:srv/SetObstacleSizeRampLike.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/srv/detail/set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/srv/detail/set_obstacle_size_ramp_like__functions.h"
#include "march_shared_msgs/srv/detail/set_obstacle_size_ramp_like__struct.h"


// Include directives for member types
// Member `obstacle_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__srv__SetObstacleSizeRampLike_Request__init(message_memory);
}

void SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_fini_function(void * message_memory)
{
  march_shared_msgs__srv__SetObstacleSizeRampLike_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_message_member_array[4] = {
  {
    "obstacle_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__srv__SetObstacleSizeRampLike_Request, obstacle_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "new_slope_up",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__srv__SetObstacleSizeRampLike_Request, new_slope_up),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "new_slope_down",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__srv__SetObstacleSizeRampLike_Request, new_slope_down),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "new_ramp_up_length",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__srv__SetObstacleSizeRampLike_Request, new_ramp_up_length),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_message_members = {
  "march_shared_msgs__srv",  // message namespace
  "SetObstacleSizeRampLike_Request",  // message name
  4,  // number of fields
  sizeof(march_shared_msgs__srv__SetObstacleSizeRampLike_Request),
  SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_message_member_array,  // message members
  SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_message_type_support_handle = {
  0,
  &SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, SetObstacleSizeRampLike_Request)() {
  if (!SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_message_type_support_handle.typesupport_identifier) {
    SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SetObstacleSizeRampLike_Request__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "march_shared_msgs/srv/detail/set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c.h"
// already included above
// #include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "march_shared_msgs/srv/detail/set_obstacle_size_ramp_like__functions.h"
// already included above
// #include "march_shared_msgs/srv/detail/set_obstacle_size_ramp_like__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__srv__SetObstacleSizeRampLike_Response__init(message_memory);
}

void SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_fini_function(void * message_memory)
{
  march_shared_msgs__srv__SetObstacleSizeRampLike_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__srv__SetObstacleSizeRampLike_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_message_members = {
  "march_shared_msgs__srv",  // message namespace
  "SetObstacleSizeRampLike_Response",  // message name
  1,  // number of fields
  sizeof(march_shared_msgs__srv__SetObstacleSizeRampLike_Response),
  SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_message_member_array,  // message members
  SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_message_type_support_handle = {
  0,
  &SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, SetObstacleSizeRampLike_Response)() {
  if (!SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_message_type_support_handle.typesupport_identifier) {
    SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SetObstacleSizeRampLike_Response__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "march_shared_msgs/srv/detail/set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers march_shared_msgs__srv__detail__set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_service_members = {
  "march_shared_msgs__srv",  // service namespace
  "SetObstacleSizeRampLike",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // march_shared_msgs__srv__detail__set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Request_message_type_support_handle,
  NULL  // response message
  // march_shared_msgs__srv__detail__set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_Response_message_type_support_handle
};

static rosidl_service_type_support_t march_shared_msgs__srv__detail__set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_service_type_support_handle = {
  0,
  &march_shared_msgs__srv__detail__set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, SetObstacleSizeRampLike_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, SetObstacleSizeRampLike_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, SetObstacleSizeRampLike)() {
  if (!march_shared_msgs__srv__detail__set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_service_type_support_handle.typesupport_identifier) {
    march_shared_msgs__srv__detail__set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)march_shared_msgs__srv__detail__set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, SetObstacleSizeRampLike_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, srv, SetObstacleSizeRampLike_Response)()->data;
  }

  return &march_shared_msgs__srv__detail__set_obstacle_size_ramp_like__rosidl_typesupport_introspection_c__SetObstacleSizeRampLike_service_type_support_handle;
}
