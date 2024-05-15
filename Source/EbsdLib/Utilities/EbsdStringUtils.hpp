/* ============================================================================
 * Copyright (c) 2020 BlueQuartz Software, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the names of any of the BlueQuartz Software contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#pragma once

#include <array>
#include <cctype>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

/*' '(0x20)space(SPC)
 * '\t'(0x09)horizontal tab(TAB)
 * '\n'(0x0a)newline(LF)
 * '\v'(0x0b)vertical tab(VT)
 * '\f'(0x0c)feed(FF)
 * '\r'(0x0d)carriage return (CR)
 */

namespace EbsdStringUtils
{
namespace detail
{
template <bool ProcessEmptyV, class InputIt, class ForwardIt, typename TokenT>
void tokenize(InputIt first, InputIt last, ForwardIt s_first, ForwardIt s_last, std::vector<TokenT>& tokens)
{
  while(true)
  {
    const auto pos = std::find_first_of(first, last, s_first, s_last);
    if(first != pos)
    {
      tokens.emplace_back(std::string{first, pos});
    }
    else
    {
      if constexpr(ProcessEmptyV)
      {
        tokens.emplace_back("");
      }
    }
    if(pos == last)
    {
      break;
    }
    first = std::next(pos);
  }
}

template <bool ConsecutiveAsEmptyV, bool EmptyInitialV, bool EmptyFinalV>
struct SplitTypeOptions
{
  static inline constexpr bool AllowConsecutiveAsEmpty = ConsecutiveAsEmptyV;
  static inline constexpr bool AllowEmptyInital = EmptyInitialV;
  static inline constexpr bool AllowEmptyFinal = EmptyFinalV;
};

using SplitIgnoreEmpty = SplitTypeOptions<false, false, false>;
using SplitAllowAll = SplitTypeOptions<true, true, true>;
using SplitNoStripIgnoreConsecutive = SplitTypeOptions<false, true, true>;
using SplitOnlyConsecutive = SplitTypeOptions<true, false, false>;
using SplitAllowEmptyLeftAnalyze = SplitTypeOptions<true, true, false>;
using SplitAllowEmptyRightAnalyze = SplitTypeOptions<true, false, true>;

template <class SplitTypeOptionsV = SplitIgnoreEmpty>
inline std::vector<std::string> optimized_split(std::string_view str, std::vector<char>&& delimiters)
{
  if(str.empty())
  {
    return {};
  }
  auto endPos = str.end();
  auto startPos = str.begin();

  std::vector<std::string> tokens;
  tokens.reserve(str.size() / 2);

  if constexpr(SplitTypeOptionsV::AllowEmptyInital)
  {
    if(std::find(delimiters.cbegin(), delimiters.cend(), str[0]) != delimiters.cend())
    {
      tokens.emplace_back("");
      startPos++;
    }
  }

  if constexpr(!SplitTypeOptionsV::AllowEmptyFinal)
  {
    if(std::find(delimiters.cbegin(), delimiters.cend(), str[str.size() - 1]) != delimiters.cend())
    {
      endPos--;
    }
  }

  if constexpr(!SplitTypeOptionsV::AllowConsecutiveAsEmpty)
  {
    tokenize<false>(startPos, endPos, delimiters.cbegin(), delimiters.cend(), tokens);
    if constexpr(SplitTypeOptionsV::AllowEmptyFinal)
    {
      if(std::find(delimiters.cbegin(), delimiters.cend(), str[str.size() - 1]) != delimiters.cend())
      {
        tokens.emplace_back("");
      }
    }
  }
  else
  {
    if constexpr(!SplitTypeOptionsV::AllowEmptyInital)
    {
      if(std::find(delimiters.cbegin(), delimiters.cend(), str[0]) != delimiters.cend())
      {
        startPos++;
      }
    }
    tokenize<true>(startPos, endPos, delimiters.cbegin(), delimiters.cend(), tokens);
  }

  tokens.shrink_to_fit();

  // No Delimiters found
  if(tokens.empty())
  {
    tokens.emplace_back(str);
  }

  return tokens;
}
} // namespace detail

inline const std::string k_Whitespaces = " \t\f\v\n\r";

using StringTokenType = std::vector<std::string>;

enum SplitType : uint8_t
{
  IgnoreEmpty,
  AllowAll,
  NoStripIgnoreConsecutive,
  OnlyConsecutive,
  AllowEmptyLeftAnalyze,
  AllowEmptyRightAnalyze
};

inline std::vector<std::string> specific_split(std::string_view str,  std::vector<char>&& delimiters, SplitType splitType)
{
  switch(splitType)
  {
  case IgnoreEmpty:
    return detail::optimized_split<detail::SplitIgnoreEmpty>(str, std::move(delimiters));
  case AllowAll:
    return detail::optimized_split<detail::SplitAllowAll>(str, std::move(delimiters));
  case NoStripIgnoreConsecutive:
    return detail::optimized_split<detail::SplitNoStripIgnoreConsecutive>(str, std::move(delimiters));
  case OnlyConsecutive:
    return detail::optimized_split<detail::SplitOnlyConsecutive>(str, std::move(delimiters));
  case AllowEmptyLeftAnalyze:
    return detail::optimized_split<detail::SplitAllowEmptyLeftAnalyze>(str, std::move(delimiters));
  case AllowEmptyRightAnalyze:
    return detail::optimized_split<detail::SplitAllowEmptyRightAnalyze>(str, std::move(delimiters));
  }

  return {};
}

inline std::vector<std::string> split(std::string_view str, std::vector<char>&& delimiters, bool consecutiveDelimiters)
{
  if(consecutiveDelimiters)
  {
    // Split Allow All was selected to match QString's base split functionality
    return detail::optimized_split<detail::SplitAllowAll>(str, std::move(delimiters));
  }
  else
  {
    return detail::optimized_split<detail::SplitIgnoreEmpty>(str, std::move(delimiters));
  }
}

inline std::vector<std::string> split(std::string_view str, char delim)
{
  return detail::optimized_split<detail::SplitIgnoreEmpty>(str, std::vector<char>{delim});
}

inline std::string replace(std::string str, const std::string& from, const std::string& to)
{
  size_t start_pos = 0;
  while((start_pos = str.find(from, start_pos)) != std::string::npos)
  {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
  return str;
}

inline std::string ltrim(const std::string& s)
{
  std::string out = s;
  if(out.empty())
  {
    return out;
  }
  std::string::size_type front = out.find_first_not_of(k_Whitespaces);
  if(front != std::string::npos)
  {
    out = out.substr(front);
  }
  else
  {
    out.clear();
  }
  return out;
}

inline std::string rtrim(const std::string& s)
{
  std::string out = s;
  if(out.empty())
  {
    return out;
  }
  std::string::size_type back = out.find_last_not_of(k_Whitespaces);
  if(back != std::string::npos)
  {
    out.erase(back + 1);
  }
  else
  {
    out.clear();
  }
  return out;
}

inline std::string trimmed(const std::string& s)
{
  std::string out = s;
  if(out.empty())
  {
    return out;
  }
  std::string::size_type back = out.find_last_not_of(k_Whitespaces);
  if(back != std::string::npos)
  {
    out.erase(back + 1);
  }
  else
  {
    out.clear();
  }
  std::string::size_type front = out.find_first_not_of(k_Whitespaces);
  if(front != std::string::npos)
  {
    out = out.substr(front);
  }
  else
  {
    out.clear();
  }
  return out;
}

inline std::string chop(const std::string& s, size_t numElements)
{
  return s.substr(0, s.size() - numElements);
}

template <typename T>
inline std::string number(T arg)
{
  std::stringstream ss;
  ss << arg;
  return ss.str();
}

inline std::string simplified(const std::string& text)
{
  if(text.empty())
  {
    return {""};
  }
  std::string out;
  out = trimmed(text);
  std::string finalString;
  for(const auto& c : out)
  {
    if(std::isspace(c) == 0)
    {
      finalString += c;
    }
    else
    {
      finalString += ' ';
    }
  }
  return finalString;
}
} // namespace EbsdStringUtils
