// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from synapse_msgs:msg/PixyVector.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__TRAITS_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "synapse_msgs/msg/detail/pixy_vector__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace synapse_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PixyVector & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: m0_x0
  {
    out << "m0_x0: ";
    rosidl_generator_traits::value_to_yaml(msg.m0_x0, out);
    out << ", ";
  }

  // member: m0_y0
  {
    out << "m0_y0: ";
    rosidl_generator_traits::value_to_yaml(msg.m0_y0, out);
    out << ", ";
  }

  // member: m0_x1
  {
    out << "m0_x1: ";
    rosidl_generator_traits::value_to_yaml(msg.m0_x1, out);
    out << ", ";
  }

  // member: m0_y1
  {
    out << "m0_y1: ";
    rosidl_generator_traits::value_to_yaml(msg.m0_y1, out);
    out << ", ";
  }

  // member: m1_x0
  {
    out << "m1_x0: ";
    rosidl_generator_traits::value_to_yaml(msg.m1_x0, out);
    out << ", ";
  }

  // member: m1_y0
  {
    out << "m1_y0: ";
    rosidl_generator_traits::value_to_yaml(msg.m1_y0, out);
    out << ", ";
  }

  // member: m1_x1
  {
    out << "m1_x1: ";
    rosidl_generator_traits::value_to_yaml(msg.m1_x1, out);
    out << ", ";
  }

  // member: m1_y1
  {
    out << "m1_y1: ";
    rosidl_generator_traits::value_to_yaml(msg.m1_y1, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PixyVector & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: m0_x0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m0_x0: ";
    rosidl_generator_traits::value_to_yaml(msg.m0_x0, out);
    out << "\n";
  }

  // member: m0_y0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m0_y0: ";
    rosidl_generator_traits::value_to_yaml(msg.m0_y0, out);
    out << "\n";
  }

  // member: m0_x1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m0_x1: ";
    rosidl_generator_traits::value_to_yaml(msg.m0_x1, out);
    out << "\n";
  }

  // member: m0_y1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m0_y1: ";
    rosidl_generator_traits::value_to_yaml(msg.m0_y1, out);
    out << "\n";
  }

  // member: m1_x0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m1_x0: ";
    rosidl_generator_traits::value_to_yaml(msg.m1_x0, out);
    out << "\n";
  }

  // member: m1_y0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m1_y0: ";
    rosidl_generator_traits::value_to_yaml(msg.m1_y0, out);
    out << "\n";
  }

  // member: m1_x1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m1_x1: ";
    rosidl_generator_traits::value_to_yaml(msg.m1_x1, out);
    out << "\n";
  }

  // member: m1_y1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m1_y1: ";
    rosidl_generator_traits::value_to_yaml(msg.m1_y1, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PixyVector & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace synapse_msgs

namespace rosidl_generator_traits
{

[[deprecated("use synapse_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const synapse_msgs::msg::PixyVector & msg,
  std::ostream & out, size_t indentation = 0)
{
  synapse_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use synapse_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const synapse_msgs::msg::PixyVector & msg)
{
  return synapse_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<synapse_msgs::msg::PixyVector>()
{
  return "synapse_msgs::msg::PixyVector";
}

template<>
inline const char * name<synapse_msgs::msg::PixyVector>()
{
  return "synapse_msgs/msg/PixyVector";
}

template<>
struct has_fixed_size<synapse_msgs::msg::PixyVector>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<synapse_msgs::msg::PixyVector>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<synapse_msgs::msg::PixyVector>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__TRAITS_HPP_
