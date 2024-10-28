// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from synapse_msgs:msg/PixyVector.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/pixy_vector__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
synapse_msgs__msg__PixyVector__init(synapse_msgs__msg__PixyVector * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    synapse_msgs__msg__PixyVector__fini(msg);
    return false;
  }
  // m0_x0
  // m0_y0
  // m0_x1
  // m0_y1
  // m1_x0
  // m1_y0
  // m1_x1
  // m1_y1
  return true;
}

void
synapse_msgs__msg__PixyVector__fini(synapse_msgs__msg__PixyVector * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // m0_x0
  // m0_y0
  // m0_x1
  // m0_y1
  // m1_x0
  // m1_y0
  // m1_x1
  // m1_y1
}

bool
synapse_msgs__msg__PixyVector__are_equal(const synapse_msgs__msg__PixyVector * lhs, const synapse_msgs__msg__PixyVector * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // m0_x0
  if (lhs->m0_x0 != rhs->m0_x0) {
    return false;
  }
  // m0_y0
  if (lhs->m0_y0 != rhs->m0_y0) {
    return false;
  }
  // m0_x1
  if (lhs->m0_x1 != rhs->m0_x1) {
    return false;
  }
  // m0_y1
  if (lhs->m0_y1 != rhs->m0_y1) {
    return false;
  }
  // m1_x0
  if (lhs->m1_x0 != rhs->m1_x0) {
    return false;
  }
  // m1_y0
  if (lhs->m1_y0 != rhs->m1_y0) {
    return false;
  }
  // m1_x1
  if (lhs->m1_x1 != rhs->m1_x1) {
    return false;
  }
  // m1_y1
  if (lhs->m1_y1 != rhs->m1_y1) {
    return false;
  }
  return true;
}

bool
synapse_msgs__msg__PixyVector__copy(
  const synapse_msgs__msg__PixyVector * input,
  synapse_msgs__msg__PixyVector * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // m0_x0
  output->m0_x0 = input->m0_x0;
  // m0_y0
  output->m0_y0 = input->m0_y0;
  // m0_x1
  output->m0_x1 = input->m0_x1;
  // m0_y1
  output->m0_y1 = input->m0_y1;
  // m1_x0
  output->m1_x0 = input->m1_x0;
  // m1_y0
  output->m1_y0 = input->m1_y0;
  // m1_x1
  output->m1_x1 = input->m1_x1;
  // m1_y1
  output->m1_y1 = input->m1_y1;
  return true;
}

synapse_msgs__msg__PixyVector *
synapse_msgs__msg__PixyVector__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__PixyVector * msg = (synapse_msgs__msg__PixyVector *)allocator.allocate(sizeof(synapse_msgs__msg__PixyVector), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(synapse_msgs__msg__PixyVector));
  bool success = synapse_msgs__msg__PixyVector__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
synapse_msgs__msg__PixyVector__destroy(synapse_msgs__msg__PixyVector * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    synapse_msgs__msg__PixyVector__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
synapse_msgs__msg__PixyVector__Sequence__init(synapse_msgs__msg__PixyVector__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__PixyVector * data = NULL;

  if (size) {
    data = (synapse_msgs__msg__PixyVector *)allocator.zero_allocate(size, sizeof(synapse_msgs__msg__PixyVector), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = synapse_msgs__msg__PixyVector__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        synapse_msgs__msg__PixyVector__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
synapse_msgs__msg__PixyVector__Sequence__fini(synapse_msgs__msg__PixyVector__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      synapse_msgs__msg__PixyVector__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

synapse_msgs__msg__PixyVector__Sequence *
synapse_msgs__msg__PixyVector__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__PixyVector__Sequence * array = (synapse_msgs__msg__PixyVector__Sequence *)allocator.allocate(sizeof(synapse_msgs__msg__PixyVector__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = synapse_msgs__msg__PixyVector__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
synapse_msgs__msg__PixyVector__Sequence__destroy(synapse_msgs__msg__PixyVector__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    synapse_msgs__msg__PixyVector__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
synapse_msgs__msg__PixyVector__Sequence__are_equal(const synapse_msgs__msg__PixyVector__Sequence * lhs, const synapse_msgs__msg__PixyVector__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!synapse_msgs__msg__PixyVector__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
synapse_msgs__msg__PixyVector__Sequence__copy(
  const synapse_msgs__msg__PixyVector__Sequence * input,
  synapse_msgs__msg__PixyVector__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(synapse_msgs__msg__PixyVector);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    synapse_msgs__msg__PixyVector * data =
      (synapse_msgs__msg__PixyVector *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!synapse_msgs__msg__PixyVector__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          synapse_msgs__msg__PixyVector__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!synapse_msgs__msg__PixyVector__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
