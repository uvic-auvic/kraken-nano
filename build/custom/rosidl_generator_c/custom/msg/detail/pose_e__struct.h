// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom:msg/PoseE.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__POSE_E__STRUCT_H_
#define CUSTOM__MSG__DETAIL__POSE_E__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pos'
#include "custom/msg/detail/point__struct.h"
// Member 'rot'
#include "custom/msg/detail/euler__struct.h"

/// Struct defined in msg/PoseE in the package custom.
typedef struct custom__msg__PoseE
{
  custom__msg__Point pos;
  custom__msg__Euler rot;
} custom__msg__PoseE;

// Struct for a sequence of custom__msg__PoseE.
typedef struct custom__msg__PoseE__Sequence
{
  custom__msg__PoseE * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom__msg__PoseE__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM__MSG__DETAIL__POSE_E__STRUCT_H_
