// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_interfaces:msg/Complex.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_interfaces/msg/complex.hpp"


#ifndef MY_INTERFACES__MSG__DETAIL__COMPLEX__BUILDER_HPP_
#define MY_INTERFACES__MSG__DETAIL__COMPLEX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_interfaces/msg/detail/complex__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_interfaces
{

namespace msg
{

namespace builder
{

class Init_Complex_imaginary
{
public:
  explicit Init_Complex_imaginary(::my_interfaces::msg::Complex & msg)
  : msg_(msg)
  {}
  ::my_interfaces::msg::Complex imaginary(::my_interfaces::msg::Complex::_imaginary_type arg)
  {
    msg_.imaginary = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_interfaces::msg::Complex msg_;
};

class Init_Complex_real
{
public:
  Init_Complex_real()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Complex_imaginary real(::my_interfaces::msg::Complex::_real_type arg)
  {
    msg_.real = std::move(arg);
    return Init_Complex_imaginary(msg_);
  }

private:
  ::my_interfaces::msg::Complex msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_interfaces::msg::Complex>()
{
  return my_interfaces::msg::builder::Init_Complex_real();
}

}  // namespace my_interfaces

#endif  // MY_INTERFACES__MSG__DETAIL__COMPLEX__BUILDER_HPP_
