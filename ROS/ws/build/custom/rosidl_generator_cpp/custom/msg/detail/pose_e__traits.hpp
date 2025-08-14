// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom:msg/PoseE.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__POSE_E__TRAITS_HPP_
#define CUSTOM__MSG__DETAIL__POSE_E__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom/msg/detail/pose_e__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pos'
#include "custom/msg/detail/point__traits.hpp"
// Member 'rot'
#include "custom/msg/detail/euler__traits.hpp"

namespace custom
{

namespace msg
{

inline void to_flow_style_yaml(
  const PoseE & msg,
  std::ostream & out)
{
  out << "{";
  // member: pos
  {
    out << "pos: ";
    to_flow_style_yaml(msg.pos, out);
    out << ", ";
  }

  // member: rot
  {
    out << "rot: ";
    to_flow_style_yaml(msg.rot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PoseE & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos:\n";
    to_block_style_yaml(msg.pos, out, indentation + 2);
  }

  // member: rot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rot:\n";
    to_block_style_yaml(msg.rot, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PoseE & msg, bool use_flow_style = false)
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
  const custom::msg::PoseE & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom::msg::PoseE & msg)
{
  return custom::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom::msg::PoseE>()
{
  return "custom::msg::PoseE";
}

template<>
inline const char * name<custom::msg::PoseE>()
{
  return "custom/msg/PoseE";
}

template<>
struct has_fixed_size<custom::msg::PoseE>
  : std::integral_constant<bool, has_fixed_size<custom::msg::Euler>::value && has_fixed_size<custom::msg::Point>::value> {};

template<>
struct has_bounded_size<custom::msg::PoseE>
  : std::integral_constant<bool, has_bounded_size<custom::msg::Euler>::value && has_bounded_size<custom::msg::Point>::value> {};

template<>
struct is_message<custom::msg::PoseE>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM__MSG__DETAIL__POSE_E__TRAITS_HPP_
