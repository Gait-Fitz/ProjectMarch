// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from march_shared_msgs:msg/GaitParameters.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__FUNCTIONS_H_
#define MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "march_shared_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "march_shared_msgs/msg/detail/gait_parameters__struct.h"

/// Initialize msg/GaitParameters message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * march_shared_msgs__msg__GaitParameters
 * )) before or use
 * march_shared_msgs__msg__GaitParameters__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
bool
march_shared_msgs__msg__GaitParameters__init(march_shared_msgs__msg__GaitParameters * msg);

/// Finalize msg/GaitParameters message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
void
march_shared_msgs__msg__GaitParameters__fini(march_shared_msgs__msg__GaitParameters * msg);

/// Create msg/GaitParameters message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * march_shared_msgs__msg__GaitParameters__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
march_shared_msgs__msg__GaitParameters *
march_shared_msgs__msg__GaitParameters__create();

/// Destroy msg/GaitParameters message.
/**
 * It calls
 * march_shared_msgs__msg__GaitParameters__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
void
march_shared_msgs__msg__GaitParameters__destroy(march_shared_msgs__msg__GaitParameters * msg);


/// Initialize array of msg/GaitParameters messages.
/**
 * It allocates the memory for the number of elements and calls
 * march_shared_msgs__msg__GaitParameters__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
bool
march_shared_msgs__msg__GaitParameters__Sequence__init(march_shared_msgs__msg__GaitParameters__Sequence * array, size_t size);

/// Finalize array of msg/GaitParameters messages.
/**
 * It calls
 * march_shared_msgs__msg__GaitParameters__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
void
march_shared_msgs__msg__GaitParameters__Sequence__fini(march_shared_msgs__msg__GaitParameters__Sequence * array);

/// Create array of msg/GaitParameters messages.
/**
 * It allocates the memory for the array and calls
 * march_shared_msgs__msg__GaitParameters__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
march_shared_msgs__msg__GaitParameters__Sequence *
march_shared_msgs__msg__GaitParameters__Sequence__create(size_t size);

/// Destroy array of msg/GaitParameters messages.
/**
 * It calls
 * march_shared_msgs__msg__GaitParameters__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_march_shared_msgs
void
march_shared_msgs__msg__GaitParameters__Sequence__destroy(march_shared_msgs__msg__GaitParameters__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__GAIT_PARAMETERS__FUNCTIONS_H_
