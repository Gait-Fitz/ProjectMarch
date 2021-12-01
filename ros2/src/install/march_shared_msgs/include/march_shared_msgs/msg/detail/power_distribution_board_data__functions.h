// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from march_shared_msgs:msg/PowerDistributionBoardData.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__FUNCTIONS_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "march_shared_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "march_shared_msgs/msg/detail/power_distribution_board_data__struct.h"

/// Initialize msg/PowerDistributionBoardData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * march_shared_msgs__msg__PowerDistributionBoardData
 * )) before or use
 * march_shared_msgs__msg__PowerDistributionBoardData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
bool
march_shared_msgs__msg__PowerDistributionBoardData__init(march_shared_msgs__msg__PowerDistributionBoardData * msg);

/// Finalize msg/PowerDistributionBoardData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
void
march_shared_msgs__msg__PowerDistributionBoardData__fini(march_shared_msgs__msg__PowerDistributionBoardData * msg);

/// Create msg/PowerDistributionBoardData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * march_shared_msgs__msg__PowerDistributionBoardData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
march_shared_msgs__msg__PowerDistributionBoardData *
march_shared_msgs__msg__PowerDistributionBoardData__create();

/// Destroy msg/PowerDistributionBoardData message.
/**
 * It calls
 * march_shared_msgs__msg__PowerDistributionBoardData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
void
march_shared_msgs__msg__PowerDistributionBoardData__destroy(march_shared_msgs__msg__PowerDistributionBoardData * msg);


/// Initialize array of msg/PowerDistributionBoardData messages.
/**
 * It allocates the memory for the number of elements and calls
 * march_shared_msgs__msg__PowerDistributionBoardData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
bool
march_shared_msgs__msg__PowerDistributionBoardData__Sequence__init(march_shared_msgs__msg__PowerDistributionBoardData__Sequence * array, size_t size);

/// Finalize array of msg/PowerDistributionBoardData messages.
/**
 * It calls
 * march_shared_msgs__msg__PowerDistributionBoardData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
void
march_shared_msgs__msg__PowerDistributionBoardData__Sequence__fini(march_shared_msgs__msg__PowerDistributionBoardData__Sequence * array);

/// Create array of msg/PowerDistributionBoardData messages.
/**
 * It allocates the memory for the array and calls
 * march_shared_msgs__msg__PowerDistributionBoardData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
march_shared_msgs__msg__PowerDistributionBoardData__Sequence *
march_shared_msgs__msg__PowerDistributionBoardData__Sequence__create(size_t size);

/// Destroy array of msg/PowerDistributionBoardData messages.
/**
 * It calls
 * march_shared_msgs__msg__PowerDistributionBoardData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
void
march_shared_msgs__msg__PowerDistributionBoardData__Sequence__destroy(march_shared_msgs__msg__PowerDistributionBoardData__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__POWER_DISTRIBUTION_BOARD_DATA__FUNCTIONS_H_
