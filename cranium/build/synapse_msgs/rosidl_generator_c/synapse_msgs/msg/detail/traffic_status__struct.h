// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from synapse_msgs:msg/TrafficStatus.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__STRUCT_H_
#define SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GREEN_LIGHT'.
enum
{
  synapse_msgs__msg__TrafficStatus__GREEN_LIGHT = 0
};

/// Constant 'YELLOW_LIGHT'.
enum
{
  synapse_msgs__msg__TrafficStatus__YELLOW_LIGHT = 1
};

/// Constant 'RED_LIGHT'.
enum
{
  synapse_msgs__msg__TrafficStatus__RED_LIGHT = 2
};

/// Struct defined in msg/TrafficStatus in the package synapse_msgs.
typedef struct synapse_msgs__msg__TrafficStatus
{
  bool stop_sign;
  uint8_t traffic_light;
} synapse_msgs__msg__TrafficStatus;

// Struct for a sequence of synapse_msgs__msg__TrafficStatus.
typedef struct synapse_msgs__msg__TrafficStatus__Sequence
{
  synapse_msgs__msg__TrafficStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} synapse_msgs__msg__TrafficStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__STRUCT_H_
