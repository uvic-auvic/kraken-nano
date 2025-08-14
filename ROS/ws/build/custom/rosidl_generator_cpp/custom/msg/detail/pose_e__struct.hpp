// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom:msg/PoseE.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM__MSG__DETAIL__POSE_E__STRUCT_HPP_
#define CUSTOM__MSG__DETAIL__POSE_E__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pos'
#include "custom/msg/detail/point__struct.hpp"
// Member 'rot'
#include "custom/msg/detail/euler__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom__msg__PoseE __attribute__((deprecated))
#else
# define DEPRECATED__custom__msg__PoseE __declspec(deprecated)
#endif

namespace custom
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseE_
{
  using Type = PoseE_<ContainerAllocator>;

  explicit PoseE_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pos(_init),
    rot(_init)
  {
    (void)_init;
  }

  explicit PoseE_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pos(_alloc, _init),
    rot(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pos_type =
    custom::msg::Point_<ContainerAllocator>;
  _pos_type pos;
  using _rot_type =
    custom::msg::Euler_<ContainerAllocator>;
  _rot_type rot;

  // setters for named parameter idiom
  Type & set__pos(
    const custom::msg::Point_<ContainerAllocator> & _arg)
  {
    this->pos = _arg;
    return *this;
  }
  Type & set__rot(
    const custom::msg::Euler_<ContainerAllocator> & _arg)
  {
    this->rot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom::msg::PoseE_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom::msg::PoseE_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom::msg::PoseE_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom::msg::PoseE_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom::msg::PoseE_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom::msg::PoseE_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom::msg::PoseE_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom::msg::PoseE_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom::msg::PoseE_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom::msg::PoseE_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom__msg__PoseE
    std::shared_ptr<custom::msg::PoseE_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom__msg__PoseE
    std::shared_ptr<custom::msg::PoseE_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseE_ & other) const
  {
    if (this->pos != other.pos) {
      return false;
    }
    if (this->rot != other.rot) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseE_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseE_

// alias to use template instance with default allocator
using PoseE =
  custom::msg::PoseE_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom

#endif  // CUSTOM__MSG__DETAIL__POSE_E__STRUCT_HPP_
