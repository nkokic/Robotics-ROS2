// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_interfaces:msg/Complex.idl
// generated code does not contain a copyright notice

#include "my_interfaces/msg/detail/complex__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_interfaces
const rosidl_type_hash_t *
my_interfaces__msg__Complex__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf9, 0x63, 0x6c, 0xea, 0xaf, 0xc9, 0x5f, 0x3e,
      0xa5, 0x87, 0x83, 0x4a, 0x70, 0x8a, 0xca, 0x64,
      0xfe, 0x0e, 0x6d, 0xb9, 0xed, 0x0b, 0xdf, 0x0e,
      0xd6, 0x8f, 0x6c, 0x54, 0x24, 0xb9, 0x49, 0xd6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char my_interfaces__msg__Complex__TYPE_NAME[] = "my_interfaces/msg/Complex";

// Define type names, field names, and default values
static char my_interfaces__msg__Complex__FIELD_NAME__real[] = "real";
static char my_interfaces__msg__Complex__FIELD_NAME__imaginary[] = "imaginary";

static rosidl_runtime_c__type_description__Field my_interfaces__msg__Complex__FIELDS[] = {
  {
    {my_interfaces__msg__Complex__FIELD_NAME__real, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_interfaces__msg__Complex__FIELD_NAME__imaginary, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_interfaces__msg__Complex__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_interfaces__msg__Complex__TYPE_NAME, 25, 25},
      {my_interfaces__msg__Complex__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 real\n"
  "float64 imaginary";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_interfaces__msg__Complex__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_interfaces__msg__Complex__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 30, 30},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_interfaces__msg__Complex__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_interfaces__msg__Complex__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
