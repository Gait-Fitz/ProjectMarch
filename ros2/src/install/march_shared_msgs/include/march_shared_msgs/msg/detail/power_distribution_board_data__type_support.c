// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from march_shared_msgs:msg/PowerDistributionBoardData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "march_shared_msgs/msg/detail/power_distribution_board_data__rosidl_typesupport_introspection_c.h"
#include "march_shared_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "march_shared_msgs/msg/detail/power_distribution_board_data__functions.h"
#include "march_shared_msgs/msg/detail/power_distribution_board_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `hv_state`
#include "march_shared_msgs/msg/high_voltage_state.h"
// Member `hv_state`
#include "march_shared_msgs/msg/detail/high_voltage_state__rosidl_typesupport_introspection_c.h"
// Member `lv_state`
#include "march_shared_msgs/msg/low_voltage_state.h"
// Member `lv_state`
#include "march_shared_msgs/msg/detail/low_voltage_state__rosidl_typesupport_introspection_c.h"
// Member `battery_state`
#include "march_shared_msgs/msg/battery_state.h"
// Member `battery_state`
#include "march_shared_msgs/msg/detail/battery_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  march_shared_msgs__msg__PowerDistributionBoardData__init(message_memory);
}

void PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_fini_function(void * message_memory)
{
  march_shared_msgs__msg__PowerDistributionBoardData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PowerDistributionBoardData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "emergency_button_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PowerDistributionBoardData, emergency_button_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pdb_current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PowerDistributionBoardData, pdb_current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hv_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PowerDistributionBoardData, hv_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stop_button_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PowerDistributionBoardData, stop_button_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lv_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PowerDistributionBoardData, lv_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(march_shared_msgs__msg__PowerDistributionBoardData, battery_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_members = {
  "march_shared_msgs__msg",  // message namespace
  "PowerDistributionBoardData",  // message name
  7,  // number of fields
  sizeof(march_shared_msgs__msg__PowerDistributionBoardData),
  PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_member_array,  // message members
  PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_init_function,  // function to initialize message memory (memory has to be allocated)
  PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_type_support_handle = {
  0,
  &PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_march_shared_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, PowerDistributionBoardData)() {
  PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, HighVoltageState)();
  PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, LowVoltageState)();
  PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, march_shared_msgs, msg, BatteryState)();
  if (!PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_type_support_handle.typesupport_identifier) {
    PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PowerDistributionBoardData__rosidl_typesupport_introspection_c__PowerDistributionBoardData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
