// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__TASK__STRUCT_HPP_
#define CUSTOM__MSG__DETAIL__TASK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom__msg__Task __attribute__((deprecated))
#else
# define DEPRECATED__custom__msg__Task __declspec(deprecated)
#endif

namespace custom
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Task_
{
  using Type = Task_<ContainerAllocator>;

  explicit Task_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->direction = "";
      this->magnitude = 0ll;
    }
  }

  explicit Task_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : type(_alloc),
    direction(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->direction = "";
      this->magnitude = 0ll;
    }
  }

  // field types and members
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _type_type type;
  using _direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _direction_type direction;
  using _magnitude_type =
    int64_t;
  _magnitude_type magnitude;

  // setters for named parameter idiom
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__magnitude(
    const int64_t & _arg)
  {
    this->magnitude = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom::msg::Task_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom::msg::Task_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom::msg::Task_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom::msg::Task_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom::msg::Task_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom::msg::Task_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom::msg::Task_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom::msg::Task_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom::msg::Task_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom::msg::Task_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom__msg__Task
    std::shared_ptr<custom::msg::Task_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom__msg__Task
    std::shared_ptr<custom::msg::Task_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Task_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    if (this->magnitude != other.magnitude) {
      return false;
    }
    return true;
  }
  bool operator!=(const Task_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Task_

// alias to use template instance with default allocator
using Task =
  custom::msg::Task_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom

#endif  // CUSTOM__MSG__DETAIL__TASK__STRUCT_HPP_
