/* ============================================================================
 * Copyright (c) 2009-2016 BlueQuartz Software, LLC
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
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * The code contained herein was partially funded by the following contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#pragma once

#include <charconv>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

#include "EbsdLib/Core/EbsdSetGetMacros.h"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/IO/EbsdHeaderEntry.h"

#ifdef EbsdLib_ENABLE_HDF5
#include "H5Support/H5Lite.h"
using namespace H5Support;
#endif

class Int32HeaderParser
{
public:
  Int32HeaderParser() = default;
  ~Int32HeaderParser() = default;
  Int32HeaderParser(const Int32HeaderParser&) = delete;            // Copy Constructor Not Implemented
  Int32HeaderParser(Int32HeaderParser&&) = delete;                 // Move Constructor Not Implemented
  Int32HeaderParser& operator=(const Int32HeaderParser&) = delete; // Copy Assignment Not Implemented
  Int32HeaderParser& operator=(Int32HeaderParser&&) = delete;      // Move Assignment Not Implemented
  int32_t parse(const std::string& bytes)
  {
    int32_t value = -1;
    auto [p, ec] = std::from_chars(bytes.data(), bytes.data() + bytes.size(), value, 10);
    if(ec == std::errc::invalid_argument || ec == std::errc::result_out_of_range)
    {
      value = -1;
    }
    return value;
  }
};

class FloatHeaderParser
{
public:
  FloatHeaderParser() = default;
  ~FloatHeaderParser() = default;
  FloatHeaderParser(const FloatHeaderParser&) = delete;            // Copy Constructor Not Implemented
  FloatHeaderParser(FloatHeaderParser&&) = delete;                 // Move Constructor Not Implemented
  FloatHeaderParser& operator=(const FloatHeaderParser&) = delete; // Copy Assignment Not Implemented
  FloatHeaderParser& operator=(FloatHeaderParser&&) = delete;      // Move Assignment Not Implemented
  float parse(const std::string& bytes)
  {
    float value = std::stof(bytes);
    return value;
  }
};

/**
 * @class CtfHeaderEntry CtfHeaderEntry.h EbsdLib/IO/HKL/CtfHeaderEntry.h
 * @brief Header entry that holds an integer or decimal type value
 *
 * @date Aug 8, 2011
 * @version 1.0
 */
template <typename T, typename ParserType>
class EbsdLib_EXPORT CtfHeaderEntry : public EbsdHeaderEntry
{

public:
  using Self = CtfHeaderEntry<T, ParserType>;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer();

  HEADERENTRY_NEW_SUPERCLASS(Self, EbsdHeaderEntry)
  /**
   * @brief Returns the name of the class for CtfHeaderEntry
   */
  std::string getNameOfClass() const;
  /**
   * @brief Returns the name of the class for CtfHeaderEntry
   */
  static std::string ClassName();

  ~CtfHeaderEntry() override = default;

  /**
   * @brief getKey
   * @return
   */
  std::string getKey() override
  {
    return m_key;
  }

#ifdef EbsdLib_ENABLE_HDF5
  std::string getHDFType() override
  {
    return H5Lite::HDFTypeForPrimitiveAsStr<T>();
  }
#endif
  /**
   * @brief parseValue
   * @param value
   */
  void parseValue(std::string& bytes) override
  {
    // Simple Naieve filter to remove European style decimals that use a comma
    for(char& v : bytes)
    {
      if(v == ',')
      {
        v = '.';
      }
    }

    m_Value = m_Parser.parse(bytes);
  }

  /**
   * @brief print
   * @param out
   */
  void print(std::ostream& out) override
  {
    out << m_key << "  " << m_Value << std::endl;
  }

  /**
   * @brief getValue
   * @return
   */
  T getValue()
  {
    return m_Value;
  }

  /**
   * @brief setValue
   * @param value
   */
  void setValue(T value)
  {
    m_Value = value;
  }

protected:
  CtfHeaderEntry(std::string key)
  : m_Value(0)
  , m_key(std::move(key))
  {
  }

  CtfHeaderEntry() = default;

private:
  T m_Value;
  std::string m_key;

  ParserType m_Parser;

public:
  CtfHeaderEntry(const CtfHeaderEntry&) = delete;            // Copy Constructor Not Implemented
  CtfHeaderEntry(CtfHeaderEntry&&) = delete;                 // Move Constructor Not Implemented
  CtfHeaderEntry& operator=(const CtfHeaderEntry&) = delete; // Copy Assignment Not Implemented
  CtfHeaderEntry& operator=(CtfHeaderEntry&&) = delete;      // Move Assignment Not Implemented
};

/**
 * @class CtfStringHeaderEntry CtfHeaderEntry.h EbsdLib/IO/HKL/CtfHeaderEntry.h
 * @brief Header entry that holds a string type value
 *
 * @date Aug 1, 2011
 * @version 1.0
 */
class CtfStringHeaderEntry : public EbsdHeaderEntry
{
public:
  using Self = CtfStringHeaderEntry;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer();

  HEADERENTRY_NEW_SUPERCLASS(CtfStringHeaderEntry, EbsdHeaderEntry)

  ~CtfStringHeaderEntry() override = default;

  std::string getKey() override
  {
    return m_key;
  }

#ifdef EbsdLib_ENABLE_HDF5
  std::string getHDFType() override
  {
    return "H5T_STRING";
  }
#endif

  void parseValue(std::string& value) override
  {
    m_Value = std::string(value);
  }

  void print(std::ostream& out) override
  {
    out << m_key << "  " << m_Value << std::endl;
  }

  std::string getValue()
  {
    return m_Value;
  }
  void setValue(const std::string& value)
  {
    m_Value = value;
  }

protected:
  CtfStringHeaderEntry(std::string key)
  : m_key(std::move(key))
  {
  }

  CtfStringHeaderEntry() = default;

private:
  std::string m_Value;
  std::string m_key;

public:
  CtfStringHeaderEntry(const CtfStringHeaderEntry&) = delete;            // Copy Constructor Not Implemented
  CtfStringHeaderEntry(CtfStringHeaderEntry&&) = delete;                 // Move Constructor Not Implemented
  CtfStringHeaderEntry& operator=(const CtfStringHeaderEntry&) = delete; // Copy Assignment Not Implemented
  CtfStringHeaderEntry& operator=(CtfStringHeaderEntry&&) = delete;      // Move Assignment Not Implemented
};
