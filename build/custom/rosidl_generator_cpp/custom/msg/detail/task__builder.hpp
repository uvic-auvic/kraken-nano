// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__TASK__BUILDER_HPP_
#define CUSTOM__MSG__DETAIL__TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom
{

namespace msg
{

namespace builder
{

class Init_Task_magnitude
{
public:
  explicit Init_Task_magnitude(::custom::msg::Task & msg)
  : msg_(msg)
  {}
  ::custom::msg::Task magnitude(::custom::msg::Task::_magnitude_type arg)
  {
    msg_.magnitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom::msg::Task msg_;
};

class Init_Task_direction
{
public:
  explicit Init_Task_direction(::custom::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_magnitude direction(::custom::msg::Task::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_Task_magnitude(msg_);
  }

private:
  ::custom::msg::Task msg_;
};

class Init_Task_type
{
public:
  Init_Task_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Task_direction type(::custom::msg::Task::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Task_direction(msg_);
  }

private:
  ::custom::msg::Task msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom::msg::Task>()
{
  return custom::msg::builder::Init_Task_type();
}

}  // namespace custom

#endif  // CUSTOM__MSG__DETAIL__TASK__BUILDER_HPP_
