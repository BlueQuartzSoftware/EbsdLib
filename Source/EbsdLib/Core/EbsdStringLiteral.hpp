#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

// #include <fmt/core.h>
// #include <fmt/xchar.h>

namespace ebsdlib
{

/**
 * @brief Returns true if the given character array is null-terminated.
 * @tparam T
 * @param string
 * @return
 */
template <class T, size_t Size>
constexpr bool HasNullTerminator(const T (&string)[Size]) noexcept
{
  return string[Size - 1] == static_cast<T>('\0');
}

/**
 * @brief BasicEbsdStringLiteral is meant to be a safe container for a string literal allowing for easy access to its size/length.
 * This class should always contain a pointer to a static compile time null-terminated string literal.
 * Typical usage will be for static string constants. At this time the constructors allow non string literals to be passed in.
 * This is undesired behavior, but there is no way to overcome this in C++17 without using character parameter packs which come with their own issues.
 * Example:
 * @code
 * static constexpr EbsdStringLiteral k_Foo = "foo";
 * @endcode
 * @tparam T Character type
 */
template <class T>
class BasicEbsdStringLiteral
{
public:
  BasicEbsdStringLiteral() = delete;

  /**
   * @brief Constructor that accepts a string literal of fixed size. Should be made consteval in C++20.
   * Requires string to be null-terminated.
   * @param string
   * @return
   */
  template <size_t Size>
  constexpr BasicEbsdStringLiteral(const T (&string)[Size])
  : m_String(string)
  , m_Size(Size)
  {
    if(!HasNullTerminator(string))
    {
      throw std::runtime_error("BasicEbsdStringLiteral must be null-terminated");
    }
  }

  ~BasicEbsdStringLiteral() noexcept = default;

  BasicEbsdStringLiteral(const BasicEbsdStringLiteral&) noexcept = default;
  BasicEbsdStringLiteral(BasicEbsdStringLiteral&&) noexcept = default;

  BasicEbsdStringLiteral& operator=(const BasicEbsdStringLiteral&) noexcept = default;
  BasicEbsdStringLiteral& operator=(BasicEbsdStringLiteral&&) noexcept = default;

  /**
   * @brief Returns the c-string pointer.
   * @return const T*
   */
  constexpr const T* c_str() const noexcept
  {
    return m_String;
  }

  /**
   * @brief Returns the size of the string literal not including the null terminator.
   * @return size_t
   */
  constexpr size_t size() const noexcept
  {
    return m_Size - 1;
  }

  /**
   * @brief Returns a view of string literal not including the null terminator.
   * @return std::basic_string_view<T>
   */
  constexpr std::basic_string_view<T> view() const
  {
    return std::basic_string_view<T>(m_String, size());
  }

  /**
   * @brief Returns a view of string literal including the null terminator.
   * @return std::basic_string_view<T>
   */
  constexpr std::basic_string_view<T> c_view() const
  {
    return std::basic_string_view<T>(m_String, m_Size);
  }

  /**
   * @brief Returns a null-terminated heap allocated string.
   * @return std::basic_string<T>
   */
  std::basic_string<T> str() const
  {
    return std::basic_string<T>(m_String, size());
  }

  /**
   * @brief Implicit conversion to std::basic_string<T>
   */
  operator std::basic_string<T>() const
  {
    return str();
  }

  /**
   * @brief Implicit conversion to std::basic_string_view<T>
   */
  operator std::basic_string_view<T>() const
  {
    return view();
  }

private:
  const T* m_String;
  size_t m_Size;
};

template <class T>
bool operator==(const std::basic_string<T>& lhs, BasicEbsdStringLiteral<T> rhs)
{
  return lhs == rhs.view();
}

template <class T>
bool operator==(BasicEbsdStringLiteral<T> lhs, const std::basic_string<T>& rhs)
{
  return rhs == lhs;
}

template <class T>
bool operator!=(const std::basic_string<T>& lhs, BasicEbsdStringLiteral<T> rhs)
{
  return lhs != rhs.view();
}

template <class T>
bool operator!=(BasicEbsdStringLiteral<T> lhs, const std::basic_string<T>& rhs)
{
  return rhs != lhs;
}

template <class T>
bool operator<(const std::basic_string<T>& lhs, BasicEbsdStringLiteral<T> rhs)
{
  return lhs < rhs.view();
}

template <class T>
bool operator<(BasicEbsdStringLiteral<T> lhs, const std::basic_string<T>& rhs)
{
  return lhs.view() < rhs;
}

template <class T>
bool operator>(const std::basic_string<T>& lhs, BasicEbsdStringLiteral<T> rhs)
{
  return lhs > rhs.view();
}

template <class T>
bool operator>(BasicEbsdStringLiteral<T> lhs, const std::basic_string<T>& rhs)
{
  return lhs.view() > rhs;
}

template <class T>
bool operator<=(const std::basic_string<T>& lhs, BasicEbsdStringLiteral<T> rhs)
{
  return lhs <= rhs.view();
}

template <class T>
bool operator<=(BasicEbsdStringLiteral<T> lhs, const std::basic_string<T>& rhs)
{
  return lhs.view() <= rhs;
}

template <class T>
bool operator>=(const std::basic_string<T>& lhs, BasicEbsdStringLiteral<T> rhs)
{
  return lhs >= rhs.view();
}

template <class T>
bool operator>=(BasicEbsdStringLiteral<T> lhs, const std::basic_string<T>& rhs)
{
  return lhs.view() >= rhs;
}

template <class T>
std::ostream& operator<<(std::ostream& os, const BasicEbsdStringLiteral<T>& lhs)
{
  os << lhs.view();
  return os;
}

using EbsdStringLiteral = BasicEbsdStringLiteral<char>;
using WEbsdStringLiteral = BasicEbsdStringLiteral<wchar_t>;
using String16Literal = BasicEbsdStringLiteral<char16_t>;
using String32Literal = BasicEbsdStringLiteral<char32_t>;
} // namespace ebsdlib

#if 0
template <class CharT>
struct fmt::formatter<nx::core::BasicEbsdStringLiteral<CharT>>
{
  static constexpr const CharT* GetFormatString()
  {
    if constexpr(std::is_same_v<CharT, char>)
    {
      return "{}";
    }
    if constexpr(std::is_same_v<CharT, wchar_t>)
    {
      return L"{}";
    }
    if constexpr(std::is_same_v<CharT, char16_t>)
    {
      return u"{}";
    }
    if constexpr(std::is_same_v<CharT, char32_t>)
    {
      return U"{}";
    }
  }

  constexpr typename basic_format_parse_context<CharT>::iterator parse(basic_format_parse_context<CharT>& ctx)
  {
    return ctx.begin();
  }

  typename buffer_context<CharT>::iterator format(const nx::core::BasicEbsdStringLiteral<CharT>& p, buffer_context<CharT>& ctx) const
  {
    static constexpr const CharT* formatStr = GetFormatString();

    return fmt::format_to(ctx.out(), formatStr, p.view());
  }
};
#endif
