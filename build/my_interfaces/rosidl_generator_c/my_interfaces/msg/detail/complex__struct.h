// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_interfaces:msg/Complex.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_interfaces/msg/complex.h"


#ifndef MY_INTERFACES__MSG__DETAIL__COMPLEX__STRUCT_H_
#define MY_INTERFACES__MSG__DETAIL__COMPLEX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Complex in the package my_interfaces.
typedef struct my_interfaces__msg__Complex
{
  double real;
  double imaginary;
} my_interfaces__msg__Complex;

// Struct for a sequence of my_interfaces__msg__Complex.
typedef struct my_interfaces__msg__Complex__Sequence
{
  my_interfaces__msg__Complex * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_interfaces__msg__Complex__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_INTERFACES__MSG__DETAIL__COMPLEX__STRUCT_H_
