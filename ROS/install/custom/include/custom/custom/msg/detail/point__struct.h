// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__POINT__STRUCT_H_
#define CUSTOM__MSG__DETAIL__POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Point in the package custom.
typedef struct custom__msg__Point
{
  double x;
  double y;
  double z;
} custom__msg__Point;

// Struct for a sequence of custom__msg__Point.
typedef struct custom__msg__Point__Sequence
{
  custom__msg__Point * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom__msg__Point__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM__MSG__DETAIL__POINT__STRUCT_H_
