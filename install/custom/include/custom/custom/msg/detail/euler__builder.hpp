// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom:msg/Euler.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__EULER__BUILDER_HPP_
#define CUSTOM__MSG__DETAIL__EULER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom/msg/detail/euler__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom
{

namespace msg
{

namespace builder
{

class Init_Euler_pitch
{
public:
  explicit Init_Euler_pitch(::custom::msg::Euler & msg)
  : msg_(msg)
  {}
  ::custom::msg::Euler pitch(::custom::msg::Euler::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom::msg::Euler msg_;
};

class Init_Euler_roll
{
public:
  explicit Init_Euler_roll(::custom::msg::Euler & msg)
  : msg_(msg)
  {}
  Init_Euler_pitch roll(::custom::msg::Euler::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_Euler_pitch(msg_);
  }

private:
  ::custom::msg::Euler msg_;
};

class Init_Euler_yaw
{
public:
  Init_Euler_yaw()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Euler_roll yaw(::custom::msg::Euler::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Euler_roll(msg_);
  }

private:
  ::custom::msg::Euler msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom::msg::Euler>()
{
  return custom::msg::builder::Init_Euler_yaw();
}

}  // namespace custom

#endif  // CUSTOM__MSG__DETAIL__EULER__BUILDER_HPP_
