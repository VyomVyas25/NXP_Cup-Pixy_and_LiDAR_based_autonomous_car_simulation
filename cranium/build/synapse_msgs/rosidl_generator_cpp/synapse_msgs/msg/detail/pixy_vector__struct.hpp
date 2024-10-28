// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from synapse_msgs:msg/PixyVector.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__STRUCT_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__synapse_msgs__msg__PixyVector __attribute__((deprecated))
#else
# define DEPRECATED__synapse_msgs__msg__PixyVector __declspec(deprecated)
#endif

namespace synapse_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PixyVector_
{
  using Type = PixyVector_<ContainerAllocator>;

  explicit PixyVector_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->m0_x0 = 0l;
      this->m0_y0 = 0l;
      this->m0_x1 = 0l;
      this->m0_y1 = 0l;
      this->m1_x0 = 0l;
      this->m1_y0 = 0l;
      this->m1_x1 = 0l;
      this->m1_y1 = 0l;
    }
  }

  explicit PixyVector_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->m0_x0 = 0l;
      this->m0_y0 = 0l;
      this->m0_x1 = 0l;
      this->m0_y1 = 0l;
      this->m1_x0 = 0l;
      this->m1_y0 = 0l;
      this->m1_x1 = 0l;
      this->m1_y1 = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _m0_x0_type =
    int32_t;
  _m0_x0_type m0_x0;
  using _m0_y0_type =
    int32_t;
  _m0_y0_type m0_y0;
  using _m0_x1_type =
    int32_t;
  _m0_x1_type m0_x1;
  using _m0_y1_type =
    int32_t;
  _m0_y1_type m0_y1;
  using _m1_x0_type =
    int32_t;
  _m1_x0_type m1_x0;
  using _m1_y0_type =
    int32_t;
  _m1_y0_type m1_y0;
  using _m1_x1_type =
    int32_t;
  _m1_x1_type m1_x1;
  using _m1_y1_type =
    int32_t;
  _m1_y1_type m1_y1;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__m0_x0(
    const int32_t & _arg)
  {
    this->m0_x0 = _arg;
    return *this;
  }
  Type & set__m0_y0(
    const int32_t & _arg)
  {
    this->m0_y0 = _arg;
    return *this;
  }
  Type & set__m0_x1(
    const int32_t & _arg)
  {
    this->m0_x1 = _arg;
    return *this;
  }
  Type & set__m0_y1(
    const int32_t & _arg)
  {
    this->m0_y1 = _arg;
    return *this;
  }
  Type & set__m1_x0(
    const int32_t & _arg)
  {
    this->m1_x0 = _arg;
    return *this;
  }
  Type & set__m1_y0(
    const int32_t & _arg)
  {
    this->m1_y0 = _arg;
    return *this;
  }
  Type & set__m1_x1(
    const int32_t & _arg)
  {
    this->m1_x1 = _arg;
    return *this;
  }
  Type & set__m1_y1(
    const int32_t & _arg)
  {
    this->m1_y1 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    synapse_msgs::msg::PixyVector_<ContainerAllocator> *;
  using ConstRawPtr =
    const synapse_msgs::msg::PixyVector_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<synapse_msgs::msg::PixyVector_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<synapse_msgs::msg::PixyVector_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::PixyVector_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::PixyVector_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::PixyVector_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::PixyVector_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<synapse_msgs::msg::PixyVector_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<synapse_msgs::msg::PixyVector_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__synapse_msgs__msg__PixyVector
    std::shared_ptr<synapse_msgs::msg::PixyVector_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__synapse_msgs__msg__PixyVector
    std::shared_ptr<synapse_msgs::msg::PixyVector_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PixyVector_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->m0_x0 != other.m0_x0) {
      return false;
    }
    if (this->m0_y0 != other.m0_y0) {
      return false;
    }
    if (this->m0_x1 != other.m0_x1) {
      return false;
    }
    if (this->m0_y1 != other.m0_y1) {
      return false;
    }
    if (this->m1_x0 != other.m1_x0) {
      return false;
    }
    if (this->m1_y0 != other.m1_y0) {
      return false;
    }
    if (this->m1_x1 != other.m1_x1) {
      return false;
    }
    if (this->m1_y1 != other.m1_y1) {
      return false;
    }
    return true;
  }
  bool operator!=(const PixyVector_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PixyVector_

// alias to use template instance with default allocator
using PixyVector =
  synapse_msgs::msg::PixyVector_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__STRUCT_HPP_
