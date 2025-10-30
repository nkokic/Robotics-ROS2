// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_interfaces:msg/Complex.idl
// generated code does not contain a copyright notice
#include "my_interfaces/msg/detail/complex__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
my_interfaces__msg__Complex__init(my_interfaces__msg__Complex * msg)
{
  if (!msg) {
    return false;
  }
  // real
  // imaginary
  return true;
}

void
my_interfaces__msg__Complex__fini(my_interfaces__msg__Complex * msg)
{
  if (!msg) {
    return;
  }
  // real
  // imaginary
}

bool
my_interfaces__msg__Complex__are_equal(const my_interfaces__msg__Complex * lhs, const my_interfaces__msg__Complex * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // real
  if (lhs->real != rhs->real) {
    return false;
  }
  // imaginary
  if (lhs->imaginary != rhs->imaginary) {
    return false;
  }
  return true;
}

bool
my_interfaces__msg__Complex__copy(
  const my_interfaces__msg__Complex * input,
  my_interfaces__msg__Complex * output)
{
  if (!input || !output) {
    return false;
  }
  // real
  output->real = input->real;
  // imaginary
  output->imaginary = input->imaginary;
  return true;
}

my_interfaces__msg__Complex *
my_interfaces__msg__Complex__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_interfaces__msg__Complex * msg = (my_interfaces__msg__Complex *)allocator.allocate(sizeof(my_interfaces__msg__Complex), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_interfaces__msg__Complex));
  bool success = my_interfaces__msg__Complex__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_interfaces__msg__Complex__destroy(my_interfaces__msg__Complex * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_interfaces__msg__Complex__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_interfaces__msg__Complex__Sequence__init(my_interfaces__msg__Complex__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_interfaces__msg__Complex * data = NULL;

  if (size) {
    data = (my_interfaces__msg__Complex *)allocator.zero_allocate(size, sizeof(my_interfaces__msg__Complex), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_interfaces__msg__Complex__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_interfaces__msg__Complex__fini(&data[i - 1]);
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
my_interfaces__msg__Complex__Sequence__fini(my_interfaces__msg__Complex__Sequence * array)
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
      my_interfaces__msg__Complex__fini(&array->data[i]);
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

my_interfaces__msg__Complex__Sequence *
my_interfaces__msg__Complex__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_interfaces__msg__Complex__Sequence * array = (my_interfaces__msg__Complex__Sequence *)allocator.allocate(sizeof(my_interfaces__msg__Complex__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_interfaces__msg__Complex__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_interfaces__msg__Complex__Sequence__destroy(my_interfaces__msg__Complex__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_interfaces__msg__Complex__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_interfaces__msg__Complex__Sequence__are_equal(const my_interfaces__msg__Complex__Sequence * lhs, const my_interfaces__msg__Complex__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_interfaces__msg__Complex__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_interfaces__msg__Complex__Sequence__copy(
  const my_interfaces__msg__Complex__Sequence * input,
  my_interfaces__msg__Complex__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_interfaces__msg__Complex);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_interfaces__msg__Complex * data =
      (my_interfaces__msg__Complex *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_interfaces__msg__Complex__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_interfaces__msg__Complex__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_interfaces__msg__Complex__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
