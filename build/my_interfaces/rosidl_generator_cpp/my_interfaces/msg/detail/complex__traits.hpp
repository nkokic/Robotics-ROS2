// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_interfaces:msg/Complex.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_interfaces/msg/complex.hpp"


#ifndef MY_INTERFACES__MSG__DETAIL__COMPLEX__TRAITS_HPP_
#define MY_INTERFACES__MSG__DETAIL__COMPLEX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_interfaces/msg/detail/complex__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Complex & msg,
  std::ostream & out)
{
  out << "{";
  // member: real
  {
    out << "real: ";
    rosidl_generator_traits::value_to_yaml(msg.real, out);
    out << ", ";
  }

  // member: imaginary
  {
    out << "imaginary: ";
    rosidl_generator_traits::value_to_yaml(msg.imaginary, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Complex & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: real
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real: ";
    rosidl_generator_traits::value_to_yaml(msg.real, out);
    out << "\n";
  }

  // member: imaginary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imaginary: ";
    rosidl_generator_traits::value_to_yaml(msg.imaginary, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Complex & msg, bool use_flow_style = false)
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

}  // namespace my_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use my_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_interfaces::msg::Complex & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_interfaces::msg::Complex & msg)
{
  return my_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_interfaces::msg::Complex>()
{
  return "my_interfaces::msg::Complex";
}

template<>
inline const char * name<my_interfaces::msg::Complex>()
{
  return "my_interfaces/msg/Complex";
}

template<>
struct has_fixed_size<my_interfaces::msg::Complex>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_interfaces::msg::Complex>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_interfaces::msg::Complex>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_INTERFACES__MSG__DETAIL__COMPLEX__TRAITS_HPP_
