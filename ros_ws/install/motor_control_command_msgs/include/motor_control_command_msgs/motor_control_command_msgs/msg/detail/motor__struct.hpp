// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_command_msgs:msg/Motor.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_COMMAND_MSGS__MSG__DETAIL__MOTOR__STRUCT_HPP_
#define MOTOR_CONTROL_COMMAND_MSGS__MSG__DETAIL__MOTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__motor_control_command_msgs__msg__Motor __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_command_msgs__msg__Motor __declspec(deprecated)
#endif

namespace motor_control_command_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Motor_
{
  using Type = Motor_<ContainerAllocator>;

  explicit Motor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0ul;
      this->target_position = 0l;
    }
  }

  explicit Motor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0ul;
      this->target_position = 0l;
    }
  }

  // field types and members
  using _index_type =
    uint32_t;
  _index_type index;
  using _target_position_type =
    int32_t;
  _target_position_type target_position;

  // setters for named parameter idiom
  Type & set__index(
    const uint32_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__target_position(
    const int32_t & _arg)
  {
    this->target_position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_command_msgs::msg::Motor_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_command_msgs::msg::Motor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_command_msgs::msg::Motor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_command_msgs::msg::Motor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_command_msgs::msg::Motor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_command_msgs::msg::Motor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_command_msgs::msg::Motor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_command_msgs::msg::Motor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_command_msgs::msg::Motor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_command_msgs::msg::Motor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_command_msgs__msg__Motor
    std::shared_ptr<motor_control_command_msgs::msg::Motor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_command_msgs__msg__Motor
    std::shared_ptr<motor_control_command_msgs::msg::Motor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Motor_ & other) const
  {
    if (this->index != other.index) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const Motor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Motor_

// alias to use template instance with default allocator
using Motor =
  motor_control_command_msgs::msg::Motor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_control_command_msgs

#endif  // MOTOR_CONTROL_COMMAND_MSGS__MSG__DETAIL__MOTOR__STRUCT_HPP_
