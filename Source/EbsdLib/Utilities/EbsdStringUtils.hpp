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

#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace EbsdStringUtils
{

using StringTokenType = std::vector<std::string>;

inline StringTokenType split(const std::string& line, char delimiter)
{
  std::stringstream ss(line);

  StringTokenType tokens;
  std::string temp_str;

  while(getline(ss, temp_str, delimiter))
  {
    tokens.push_back(temp_str);
  }
  return tokens;
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
  return std::regex_replace(s, std::regex("^\\s+"), std::string(""));
}

inline std::string rtrim(const std::string& s)
{
  return std::regex_replace(s, std::regex("\\s+$"), std::string(""));
}

inline std::string trimmed(const std::string& s)
{
  return ltrim(rtrim(s));
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
  std::string out;
  out = trimmed(text);
  char ch = 'a';
  std::string finalString;
  for(const auto& c : text)
  {
    if(std::isspace(c) == 0)
    {
      finalString += c;
    }
    else if(std::isspace(ch) == 0)
    {
      finalString += c;
    }
    ch = c;
  }
  return finalString;
}

} // namespace EbsdStringUtils
