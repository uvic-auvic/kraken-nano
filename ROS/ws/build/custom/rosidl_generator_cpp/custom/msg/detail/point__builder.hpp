// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__POINT__BUILDER_HPP_
#define CUSTOM__MSG__DETAIL__POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom/msg/detail/point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom
{

namespace msg
{

namespace builder
{

class Init_Point_z
{
public:
  explicit Init_Point_z(::custom::msg::Point & msg)
  : msg_(msg)
  {}
  ::custom::msg::Point z(::custom::msg::Point::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom::msg::Point msg_;
};

class Init_Point_y
{
public:
  explicit Init_Point_y(::custom::msg::Point & msg)
  : msg_(msg)
  {}
  Init_Point_z y(::custom::msg::Point::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Point_z(msg_);
  }

private:
  ::custom::msg::Point msg_;
};

class Init_Point_x
{
public:
  Init_Point_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Point_y x(::custom::msg::Point::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Point_y(msg_);
  }

private:
  ::custom::msg::Point msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom::msg::Point>()
{
  return custom::msg::builder::Init_Point_x();
}

}  // namespace custom

#endif  // CUSTOM__MSG__DETAIL__POINT__BUILDER_HPP_
