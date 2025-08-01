// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__TASK__STRUCT_H_
#define CUSTOM__MSG__DETAIL__TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
// Member 'direction'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Task in the package custom.
typedef struct custom__msg__Task
{
  rosidl_runtime_c__String type;
  rosidl_runtime_c__String direction;
  int64_t magnitude;
} custom__msg__Task;

// Struct for a sequence of custom__msg__Task.
typedef struct custom__msg__Task__Sequence
{
  custom__msg__Task * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom__msg__Task__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM__MSG__DETAIL__TASK__STRUCT_H_
