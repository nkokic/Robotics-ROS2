// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from my_interfaces:msg/Complex.idl
// generated code does not contain a copyright notice
#include "my_interfaces/msg/detail/complex__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "my_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "my_interfaces/msg/detail/complex__struct.h"
#include "my_interfaces/msg/detail/complex__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Complex__ros_msg_type = my_interfaces__msg__Complex;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_interfaces
bool cdr_serialize_my_interfaces__msg__Complex(
  const my_interfaces__msg__Complex * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: real
  {
    cdr << ros_message->real;
  }

  // Field name: imaginary
  {
    cdr << ros_message->imaginary;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_interfaces
bool cdr_deserialize_my_interfaces__msg__Complex(
  eprosima::fastcdr::Cdr & cdr,
  my_interfaces__msg__Complex * ros_message)
{
  // Field name: real
  {
    cdr >> ros_message->real;
  }

  // Field name: imaginary
  {
    cdr >> ros_message->imaginary;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_interfaces
size_t get_serialized_size_my_interfaces__msg__Complex(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Complex__ros_msg_type * ros_message = static_cast<const _Complex__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: real
  {
    size_t item_size = sizeof(ros_message->real);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imaginary
  {
    size_t item_size = sizeof(ros_message->imaginary);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_interfaces
size_t max_serialized_size_my_interfaces__msg__Complex(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: real
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: imaginary
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = my_interfaces__msg__Complex;
    is_plain =
      (
      offsetof(DataType, imaginary) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_interfaces
bool cdr_serialize_key_my_interfaces__msg__Complex(
  const my_interfaces__msg__Complex * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: real
  {
    cdr << ros_message->real;
  }

  // Field name: imaginary
  {
    cdr << ros_message->imaginary;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_interfaces
size_t get_serialized_size_key_my_interfaces__msg__Complex(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Complex__ros_msg_type * ros_message = static_cast<const _Complex__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: real
  {
    size_t item_size = sizeof(ros_message->real);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imaginary
  {
    size_t item_size = sizeof(ros_message->imaginary);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_interfaces
size_t max_serialized_size_key_my_interfaces__msg__Complex(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: real
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: imaginary
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = my_interfaces__msg__Complex;
    is_plain =
      (
      offsetof(DataType, imaginary) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Complex__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const my_interfaces__msg__Complex * ros_message = static_cast<const my_interfaces__msg__Complex *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_my_interfaces__msg__Complex(ros_message, cdr);
}

static bool _Complex__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  my_interfaces__msg__Complex * ros_message = static_cast<my_interfaces__msg__Complex *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_my_interfaces__msg__Complex(cdr, ros_message);
}

static uint32_t _Complex__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_my_interfaces__msg__Complex(
      untyped_ros_message, 0));
}

static size_t _Complex__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_my_interfaces__msg__Complex(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Complex = {
  "my_interfaces::msg",
  "Complex",
  _Complex__cdr_serialize,
  _Complex__cdr_deserialize,
  _Complex__get_serialized_size,
  _Complex__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Complex__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Complex,
  get_message_typesupport_handle_function,
  &my_interfaces__msg__Complex__get_type_hash,
  &my_interfaces__msg__Complex__get_type_description,
  &my_interfaces__msg__Complex__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, my_interfaces, msg, Complex)() {
  return &_Complex__type_support;
}

#if defined(__cplusplus)
}
#endif
