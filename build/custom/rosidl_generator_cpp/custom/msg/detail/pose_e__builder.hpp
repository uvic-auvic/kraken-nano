// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom:msg/PoseE.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__POSE_E__BUILDER_HPP_
#define CUSTOM__MSG__DETAIL__POSE_E__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom/msg/detail/pose_e__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom
{

namespace msg
{

namespace builder
{

class Init_PoseE_rot
{
public:
  explicit Init_PoseE_rot(::custom::msg::PoseE & msg)
  : msg_(msg)
  {}
  ::custom::msg::PoseE rot(::custom::msg::PoseE::_rot_type arg)
  {
    msg_.rot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom::msg::PoseE msg_;
};

class Init_PoseE_pos
{
public:
  Init_PoseE_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseE_rot pos(::custom::msg::PoseE::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_PoseE_rot(msg_);
  }

private:
  ::custom::msg::PoseE msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom::msg::PoseE>()
{
  return custom::msg::builder::Init_PoseE_pos();
}

}  // namespace custom

#endif  // CUSTOM__MSG__DETAIL__POSE_E__BUILDER_HPP_
