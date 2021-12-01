// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from march_shared_msgs:msg/PowerDistributionBoardData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__STRUCT_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'hv_state'
#include "march_shared_msgs/msg/detail/high_voltage_state__struct.h"
// Member 'lv_state'
#include "march_shared_msgs/msg/detail/low_voltage_state__struct.h"
// Member 'battery_state'
#include "march_shared_msgs/msg/detail/battery_state__struct.h"

// Struct defined in msg/PowerDistributionBoardData in the package march_shared_msgs.
typedef struct march_shared_msgs__msg__PowerDistributionBoardData
{
  std_msgs__msg__Header header;
  uint32_t emergency_button_state;
  float pdb_current;
  march_shared_msgs__msg__HighVoltageState hv_state;
  uint32_t stop_button_state;
  march_shared_msgs__msg__LowVoltageState lv_state;
  march_shared_msgs__msg__BatteryState battery_state;
} march_shared_msgs__msg__PowerDistributionBoardData;

// Struct for a sequence of march_shared_msgs__msg__PowerDistributionBoardData.
typedef struct march_shared_msgs__msg__PowerDistributionBoardData__Sequence
{
  march_shared_msgs__msg__PowerDistributionBoardData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} march_shared_msgs__msg__PowerDistributionBoardData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__STRUCT_H_
