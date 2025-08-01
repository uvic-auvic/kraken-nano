// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__TASK__TRAITS_HPP_
#define CUSTOM__MSG__DETAIL__TASK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom
{

namespace msg
{

inline void to_flow_style_yaml(
  const Task & msg,
  std::ostream & out)
{
  out << "{";
  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << ", ";
  }

  // member: magnitude
  {
    out << "magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.magnitude, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Task & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << "\n";
  }

  // member: magnitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.magnitude, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Task & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom

namespace rosidl_generator_traits
{

[[deprecated("use custom::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom::msg::Task & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom::msg::Task & msg)
{
  return custom::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom::msg::Task>()
{
  return "custom::msg::Task";
}

template<>
inline const char * name<custom::msg::Task>()
{
  return "custom/msg/Task";
}

template<>
struct has_fixed_size<custom::msg::Task>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom::msg::Task>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom::msg::Task>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM__MSG__DETAIL__TASK__TRAITS_HPP_
