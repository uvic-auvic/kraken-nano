// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom:msg/Euler.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__EULER__STRUCT_HPP_
#define CUSTOM__MSG__DETAIL__EULER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom__msg__Euler __attribute__((deprecated))
#else
# define DEPRECATED__custom__msg__Euler __declspec(deprecated)
#endif

namespace custom
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Euler_
{
  using Type = Euler_<ContainerAllocator>;

  explicit Euler_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
    }
  }

  explicit Euler_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
    }
  }

  // field types and members
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _roll_type =
    double;
  _roll_type roll;
  using _pitch_type =
    double;
  _pitch_type pitch;

  // setters for named parameter idiom
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom::msg::Euler_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom::msg::Euler_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom::msg::Euler_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom::msg::Euler_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom::msg::Euler_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom::msg::Euler_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom::msg::Euler_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom::msg::Euler_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom::msg::Euler_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom::msg::Euler_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom__msg__Euler
    std::shared_ptr<custom::msg::Euler_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom__msg__Euler
    std::shared_ptr<custom::msg::Euler_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Euler_ & other) const
  {
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    return true;
  }
  bool operator!=(const Euler_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Euler_

// alias to use template instance with default allocator
using Euler =
  custom::msg::Euler_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom

#endif  // CUSTOM__MSG__DETAIL__EULER__STRUCT_HPP_
