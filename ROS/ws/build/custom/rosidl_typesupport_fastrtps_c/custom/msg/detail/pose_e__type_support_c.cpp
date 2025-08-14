// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from custom:msg/PoseE.idl
// generated code does not contain a copyright notice
#include "custom/msg/detail/pose_e__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "custom/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "custom/msg/detail/pose_e__struct.h"
#include "custom/msg/detail/pose_e__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "custom/msg/detail/euler__functions.h"  // rot
#include "custom/msg/detail/point__functions.h"  // pos

// forward declare type support functions
size_t get_serialized_size_custom__msg__Euler(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_custom__msg__Euler(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom, msg, Euler)();
size_t get_serialized_size_custom__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_custom__msg__Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom, msg, Point)();


using _PoseE__ros_msg_type = custom__msg__PoseE;

static bool _PoseE__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PoseE__ros_msg_type * ros_message = static_cast<const _PoseE__ros_msg_type *>(untyped_ros_message);
  // Field name: pos
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, custom, msg, Point
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pos, cdr))
    {
      return false;
    }
  }

  // Field name: rot
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, custom, msg, Euler
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->rot, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _PoseE__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PoseE__ros_msg_type * ros_message = static_cast<_PoseE__ros_msg_type *>(untyped_ros_message);
  // Field name: pos
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, custom, msg, Point
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pos))
    {
      return false;
    }
  }

  // Field name: rot
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, custom, msg, Euler
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->rot))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom
size_t get_serialized_size_custom__msg__PoseE(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PoseE__ros_msg_type * ros_message = static_cast<const _PoseE__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name pos

  current_alignment += get_serialized_size_custom__msg__Point(
    &(ros_message->pos), current_alignment);
  // field.name rot

  current_alignment += get_serialized_size_custom__msg__Euler(
    &(ros_message->rot), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _PoseE__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_custom__msg__PoseE(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom
size_t max_serialized_size_custom__msg__PoseE(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: pos
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_custom__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: rot
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_custom__msg__Euler(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = custom__msg__PoseE;
    is_plain =
      (
      offsetof(DataType, rot) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PoseE__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_custom__msg__PoseE(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PoseE = {
  "custom::msg",
  "PoseE",
  _PoseE__cdr_serialize,
  _PoseE__cdr_deserialize,
  _PoseE__get_serialized_size,
  _PoseE__max_serialized_size
};

static rosidl_message_type_support_t _PoseE__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PoseE,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom, msg, PoseE)() {
  return &_PoseE__type_support;
}

#if defined(__cplusplus)
}
#endif
