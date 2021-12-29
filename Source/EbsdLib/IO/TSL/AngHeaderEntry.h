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

#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include "EbsdLib/Core/EbsdSetGetMacros.h"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/IO/EbsdHeaderEntry.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

#ifdef EbsdLib_ENABLE_HDF5
#include "H5Support/H5Lite.h"
using namespace H5Support;
#endif

/**
 * @class AngHeaderEntry AngHeaderEntry.h EbsdLib/IO/TSL/AngHeaderEntry.h
 * @brief Header entry that holds an integer or decimal type value
 *
 * @date Aug 8, 2011
 * @version 1.0
 */
template <typename T>
class EbsdLib_EXPORT AngHeaderEntry : public EbsdHeaderEntry
{

public:
  using Self = AngHeaderEntry<T>;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer();

  HEADERENTRY_NEW_SUPERCLASS(AngHeaderEntry<T>, EbsdHeaderEntry)

  ~AngHeaderEntry() override = default;

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

  void parseValue(std::string& value) override
  {
    if(value[0] == ':')
    {
      value = value.substr(1);
    } // move past the ":" character
    std::stringstream ss(value);
    ss >> m_value;
  }
  void print(std::ostream& out) override
  {
    out << m_key << "  " << m_value << std::endl;
  }

  T getValue()
  {
    return m_value;
  }
  void setValue(T value)
  {
    m_value = value;
  }

protected:
  AngHeaderEntry(std::string key)
  : m_key(std::move(key))
  {
  }

  AngHeaderEntry() = default;

private:
  T m_value = static_cast<T>(0);
  std::string m_key;

public:
  AngHeaderEntry(const AngHeaderEntry&) = delete;            // Copy Constructor Not Implemented
  AngHeaderEntry(AngHeaderEntry&&) = delete;                 // Move Constructor Not Implemented
  AngHeaderEntry& operator=(const AngHeaderEntry&) = delete; // Copy Assignment Not Implemented
  AngHeaderEntry& operator=(AngHeaderEntry&&) = delete;      // Move Assignment Not Implemented
};

/**
 * @class AngStringHeaderEntry AngStringHeaderEntry.h EbsdLib/IO/TSL/AngHeaderEntry.h
 * @brief Header entry that holds a string type value
 *
 * @date Aug 1, 2011
 * @version 1.0
 */
class AngStringHeaderEntry : public EbsdHeaderEntry
{
public:
  using Self = AngStringHeaderEntry;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer();

  HEADERENTRY_NEW_SUPERCLASS(AngStringHeaderEntry, EbsdHeaderEntry)

  ~AngStringHeaderEntry() override = default;

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
    if(value[0] == ':')
    {
      value = value.substr(1);
    }
    value = EbsdStringUtils::trimmed(value); // remove leading/trailing white space
    m_value = std::string(value);
  }
  void print(std::ostream& out) override
  {
    out << m_key << "  " << m_value << std::endl;
  }

  std::string getValue()
  {
    return m_value;
  }
  void setValue(const std::string& value)
  {
    m_value = value;
  }

protected:
  AngStringHeaderEntry(std::string key)
  : m_key(std::move(key))
  {
  }

  AngStringHeaderEntry() = default;

private:
  std::string m_value;
  std::string m_key;

public:
  AngStringHeaderEntry(const AngStringHeaderEntry&) = delete;            // Copy Constructor Not Implemented
  AngStringHeaderEntry(AngStringHeaderEntry&&) = delete;                 // Move Constructor Not Implemented
  AngStringHeaderEntry& operator=(const AngStringHeaderEntry&) = delete; // Copy Assignment Not Implemented
  AngStringHeaderEntry& operator=(AngStringHeaderEntry&&) = delete;      // Move Assignment Not Implemented
};
