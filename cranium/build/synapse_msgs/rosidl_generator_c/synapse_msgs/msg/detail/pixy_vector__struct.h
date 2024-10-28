// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from synapse_msgs:msg/PixyVector.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__STRUCT_H_
#define SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/PixyVector in the package synapse_msgs.
typedef struct synapse_msgs__msg__PixyVector
{
  std_msgs__msg__Header header;
  /// First vector coordinates
  int32_t m0_x0;
  int32_t m0_y0;
  int32_t m0_x1;
  int32_t m0_y1;
  /// Second vector coordinates
  int32_t m1_x0;
  int32_t m1_y0;
  int32_t m1_x1;
  int32_t m1_y1;
} synapse_msgs__msg__PixyVector;

// Struct for a sequence of synapse_msgs__msg__PixyVector.
typedef struct synapse_msgs__msg__PixyVector__Sequence
{
  synapse_msgs__msg__PixyVector * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} synapse_msgs__msg__PixyVector__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__PIXY_VECTOR__STRUCT_H_
