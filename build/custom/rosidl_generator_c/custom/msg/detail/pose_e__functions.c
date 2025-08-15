// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom:msg/PoseE.idl
// generated code does not contain a copyright notice
#include "custom/msg/detail/pose_e__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pos`
#include "custom/msg/detail/point__functions.h"
// Member `rot`
#include "custom/msg/detail/euler__functions.h"

bool
custom__msg__PoseE__init(custom__msg__PoseE * msg)
{
  if (!msg) {
    return false;
  }
  // pos
  if (!custom__msg__Point__init(&msg->pos)) {
    custom__msg__PoseE__fini(msg);
    return false;
  }
  // rot
  if (!custom__msg__Euler__init(&msg->rot)) {
    custom__msg__PoseE__fini(msg);
    return false;
  }
  return true;
}

void
custom__msg__PoseE__fini(custom__msg__PoseE * msg)
{
  if (!msg) {
    return;
  }
  // pos
  custom__msg__Point__fini(&msg->pos);
  // rot
  custom__msg__Euler__fini(&msg->rot);
}

bool
custom__msg__PoseE__are_equal(const custom__msg__PoseE * lhs, const custom__msg__PoseE * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pos
  if (!custom__msg__Point__are_equal(
      &(lhs->pos), &(rhs->pos)))
  {
    return false;
  }
  // rot
  if (!custom__msg__Euler__are_equal(
      &(lhs->rot), &(rhs->rot)))
  {
    return false;
  }
  return true;
}

bool
custom__msg__PoseE__copy(
  const custom__msg__PoseE * input,
  custom__msg__PoseE * output)
{
  if (!input || !output) {
    return false;
  }
  // pos
  if (!custom__msg__Point__copy(
      &(input->pos), &(output->pos)))
  {
    return false;
  }
  // rot
  if (!custom__msg__Euler__copy(
      &(input->rot), &(output->rot)))
  {
    return false;
  }
  return true;
}

custom__msg__PoseE *
custom__msg__PoseE__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom__msg__PoseE * msg = (custom__msg__PoseE *)allocator.allocate(sizeof(custom__msg__PoseE), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom__msg__PoseE));
  bool success = custom__msg__PoseE__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom__msg__PoseE__destroy(custom__msg__PoseE * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom__msg__PoseE__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom__msg__PoseE__Sequence__init(custom__msg__PoseE__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom__msg__PoseE * data = NULL;

  if (size) {
    data = (custom__msg__PoseE *)allocator.zero_allocate(size, sizeof(custom__msg__PoseE), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom__msg__PoseE__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom__msg__PoseE__fini(&data[i - 1]);
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
custom__msg__PoseE__Sequence__fini(custom__msg__PoseE__Sequence * array)
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
      custom__msg__PoseE__fini(&array->data[i]);
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

custom__msg__PoseE__Sequence *
custom__msg__PoseE__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom__msg__PoseE__Sequence * array = (custom__msg__PoseE__Sequence *)allocator.allocate(sizeof(custom__msg__PoseE__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom__msg__PoseE__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom__msg__PoseE__Sequence__destroy(custom__msg__PoseE__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom__msg__PoseE__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom__msg__PoseE__Sequence__are_equal(const custom__msg__PoseE__Sequence * lhs, const custom__msg__PoseE__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom__msg__PoseE__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom__msg__PoseE__Sequence__copy(
  const custom__msg__PoseE__Sequence * input,
  custom__msg__PoseE__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom__msg__PoseE);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom__msg__PoseE * data =
      (custom__msg__PoseE *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom__msg__PoseE__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom__msg__PoseE__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom__msg__PoseE__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
