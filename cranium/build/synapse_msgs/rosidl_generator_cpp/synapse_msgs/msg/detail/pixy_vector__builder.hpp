// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from synapse_msgs:msg/PixyVector.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__BUILDER_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "synapse_msgs/msg/detail/pixy_vector__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace synapse_msgs
{

namespace msg
{

namespace builder
{

class Init_PixyVector_m1_y1
{
public:
  explicit Init_PixyVector_m1_y1(::synapse_msgs::msg::PixyVector & msg)
  : msg_(msg)
  {}
  ::synapse_msgs::msg::PixyVector m1_y1(::synapse_msgs::msg::PixyVector::_m1_y1_type arg)
  {
    msg_.m1_y1 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::synapse_msgs::msg::PixyVector msg_;
};

class Init_PixyVector_m1_x1
{
public:
  explicit Init_PixyVector_m1_x1(::synapse_msgs::msg::PixyVector & msg)
  : msg_(msg)
  {}
  Init_PixyVector_m1_y1 m1_x1(::synapse_msgs::msg::PixyVector::_m1_x1_type arg)
  {
    msg_.m1_x1 = std::move(arg);
    return Init_PixyVector_m1_y1(msg_);
  }

private:
  ::synapse_msgs::msg::PixyVector msg_;
};

class Init_PixyVector_m1_y0
{
public:
  explicit Init_PixyVector_m1_y0(::synapse_msgs::msg::PixyVector & msg)
  : msg_(msg)
  {}
  Init_PixyVector_m1_x1 m1_y0(::synapse_msgs::msg::PixyVector::_m1_y0_type arg)
  {
    msg_.m1_y0 = std::move(arg);
    return Init_PixyVector_m1_x1(msg_);
  }

private:
  ::synapse_msgs::msg::PixyVector msg_;
};

class Init_PixyVector_m1_x0
{
public:
  explicit Init_PixyVector_m1_x0(::synapse_msgs::msg::PixyVector & msg)
  : msg_(msg)
  {}
  Init_PixyVector_m1_y0 m1_x0(::synapse_msgs::msg::PixyVector::_m1_x0_type arg)
  {
    msg_.m1_x0 = std::move(arg);
    return Init_PixyVector_m1_y0(msg_);
  }

private:
  ::synapse_msgs::msg::PixyVector msg_;
};

class Init_PixyVector_m0_y1
{
public:
  explicit Init_PixyVector_m0_y1(::synapse_msgs::msg::PixyVector & msg)
  : msg_(msg)
  {}
  Init_PixyVector_m1_x0 m0_y1(::synapse_msgs::msg::PixyVector::_m0_y1_type arg)
  {
    msg_.m0_y1 = std::move(arg);
    return Init_PixyVector_m1_x0(msg_);
  }

private:
  ::synapse_msgs::msg::PixyVector msg_;
};

class Init_PixyVector_m0_x1
{
public:
  explicit Init_PixyVector_m0_x1(::synapse_msgs::msg::PixyVector & msg)
  : msg_(msg)
  {}
  Init_PixyVector_m0_y1 m0_x1(::synapse_msgs::msg::PixyVector::_m0_x1_type arg)
  {
    msg_.m0_x1 = std::move(arg);
    return Init_PixyVector_m0_y1(msg_);
  }

private:
  ::synapse_msgs::msg::PixyVector msg_;
};

class Init_PixyVector_m0_y0
{
public:
  explicit Init_PixyVector_m0_y0(::synapse_msgs::msg::PixyVector & msg)
  : msg_(msg)
  {}
  Init_PixyVector_m0_x1 m0_y0(::synapse_msgs::msg::PixyVector::_m0_y0_type arg)
  {
    msg_.m0_y0 = std::move(arg);
    return Init_PixyVector_m0_x1(msg_);
  }

private:
  ::synapse_msgs::msg::PixyVector msg_;
};

class Init_PixyVector_m0_x0
{
public:
  explicit Init_PixyVector_m0_x0(::synapse_msgs::msg::PixyVector & msg)
  : msg_(msg)
  {}
  Init_PixyVector_m0_y0 m0_x0(::synapse_msgs::msg::PixyVector::_m0_x0_type arg)
  {
    msg_.m0_x0 = std::move(arg);
    return Init_PixyVector_m0_y0(msg_);
  }

private:
  ::synapse_msgs::msg::PixyVector msg_;
};

class Init_PixyVector_header
{
public:
  Init_PixyVector_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PixyVector_m0_x0 header(::synapse_msgs::msg::PixyVector::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PixyVector_m0_x0(msg_);
  }

private:
  ::synapse_msgs::msg::PixyVector msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::synapse_msgs::msg::PixyVector>()
{
  return synapse_msgs::msg::builder::Init_PixyVector_header();
}

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__BUILDER_HPP_
