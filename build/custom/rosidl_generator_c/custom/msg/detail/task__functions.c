// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom:msg/Task.idl
// generated code does not contain a copyright notice
#include "custom/msg/detail/task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `type`
// Member `direction`
#include "rosidl_runtime_c/string_functions.h"

bool
custom__msg__Task__init(custom__msg__Task * msg)
{
  if (!msg) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    custom__msg__Task__fini(msg);
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__init(&msg->direction)) {
    custom__msg__Task__fini(msg);
    return false;
  }
  // magnitude
  return true;
}

void
custom__msg__Task__fini(custom__msg__Task * msg)
{
  if (!msg) {
    return;
  }
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // direction
  rosidl_runtime_c__String__fini(&msg->direction);
  // magnitude
}

bool
custom__msg__Task__are_equal(const custom__msg__Task * lhs, const custom__msg__Task * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->direction), &(rhs->direction)))
  {
    return false;
  }
  // magnitude
  if (lhs->magnitude != rhs->magnitude) {
    return false;
  }
  return true;
}

bool
custom__msg__Task__copy(
  const custom__msg__Task * input,
  custom__msg__Task * output)
{
  if (!input || !output) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__copy(
      &(input->direction), &(output->direction)))
  {
    return false;
  }
  // magnitude
  output->magnitude = input->magnitude;
  return true;
}

custom__msg__Task *
custom__msg__Task__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom__msg__Task * msg = (custom__msg__Task *)allocator.allocate(sizeof(custom__msg__Task), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom__msg__Task));
  bool success = custom__msg__Task__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom__msg__Task__destroy(custom__msg__Task * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom__msg__Task__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom__msg__Task__Sequence__init(custom__msg__Task__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom__msg__Task * data = NULL;

  if (size) {
    data = (custom__msg__Task *)allocator.zero_allocate(size, sizeof(custom__msg__Task), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom__msg__Task__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom__msg__Task__fini(&data[i - 1]);
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
custom__msg__Task__Sequence__fini(custom__msg__Task__Sequence * array)
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
      custom__msg__Task__fini(&array->data[i]);
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

custom__msg__Task__Sequence *
custom__msg__Task__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom__msg__Task__Sequence * array = (custom__msg__Task__Sequence *)allocator.allocate(sizeof(custom__msg__Task__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom__msg__Task__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom__msg__Task__Sequence__destroy(custom__msg__Task__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom__msg__Task__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom__msg__Task__Sequence__are_equal(const custom__msg__Task__Sequence * lhs, const custom__msg__Task__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom__msg__Task__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom__msg__Task__Sequence__copy(
  const custom__msg__Task__Sequence * input,
  custom__msg__Task__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom__msg__Task);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom__msg__Task * data =
      (custom__msg__Task *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom__msg__Task__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom__msg__Task__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom__msg__Task__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
