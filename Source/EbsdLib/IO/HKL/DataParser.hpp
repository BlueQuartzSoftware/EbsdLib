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

#include <string>

#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/Core/EbsdSetGetMacros.h"

class DataParser
{
public:
  using Self = DataParser;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer()
  {
    return Pointer(static_cast<Self*>(nullptr));
  }

  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  std::string getNameOfClass() const
  {
    return std::string("DataParser");
  }
  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  static std::string ClassName()
  {
    return std::string("DataParser");
  }

  /**
   * @brief IsA
   * @return
   */
  virtual int32_t IsA() const
  {
    return 0;
  }

  virtual ~DataParser() = default;

  virtual bool allocateArray(size_t numberOfElements)
  {
    (void)(numberOfElements);
    return false;
  }
  virtual void* getVoidPointer()
  {
    return nullptr;
  }
  virtual void setVoidPointer(void* p)
  {
  }

  EBSD_INSTANCE_PROPERTY(bool, ManageMemory)
  EBSD_INSTANCE_PROPERTY(size_t, Size)
  EBSD_INSTANCE_STRING_PROPERTY(ColumnName)
  EBSD_INSTANCE_PROPERTY(int, ColumnIndex)

  virtual void parse(const std::string& token, size_t index)
  {
  }

  virtual EbsdLib::NumericTypes::Type getNumericType() const = 0;

protected:
  DataParser()
  : m_ManageMemory(false)
  , m_Size(0)
  , m_ColumnName("")
  , m_ColumnIndex(0)
  {
  }

public:
  DataParser(const DataParser&) = delete;            // Copy Constructor Not Implemented
  DataParser(DataParser&&) = delete;                 // Move Constructor Not Implemented
  DataParser& operator=(const DataParser&) = delete; // Copy Assignment Not Implemented
  DataParser& operator=(DataParser&&) = delete;      // Move Assignment Not Implemented
};
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
class UInt8Parser : public DataParser
{
public:
  using Self = UInt8Parser;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer()
  {
    return Pointer(static_cast<Self*>(nullptr));
  }

  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  std::string getNameOfClass() const
  {
    return std::string("UInt8Parser");
  }
  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  static std::string ClassName()
  {
    return std::string("UInt8Parser");
  }

  EbsdLib::NumericTypes::Type getNumericType() const override
  {
    return EbsdLib::NumericTypes::Type::UInt8;
  }

  int32_t IsA() const override
  {
    return 1;
  }

  static Pointer New(uint8_t* ptr, size_t size, const std::string& name, int index)
  {
    Pointer sharedPtr(new UInt8Parser(ptr, size, name, index));
    return sharedPtr;
  }

  ~UInt8Parser() override
  {
    if(m_Ptr != nullptr && getManageMemory())
    {
      delete[] m_Ptr;
      m_Ptr = nullptr;
    }
  }

  void setPtr(uint8_t* value)
  {
    this->m_Ptr = value;
  }
  uint8_t* getPtr()
  {
    return m_Ptr;
  }

  bool allocateArray(size_t numberOfElements) override
  {
    m_Ptr = new(std::nothrow) uint8_t[numberOfElements]();
    return (m_Ptr != nullptr);
  }

  void* getVoidPointer() override
  {
    return reinterpret_cast<void*>(m_Ptr);
  }
  void setVoidPointer(void* p) override
  {
    m_Ptr = reinterpret_cast<uint8_t*>(p);
  }

  uint8_t* getPointer(size_t offset)
  {
    return m_Ptr + offset;
  }

  void parse(const std::string& token, size_t index) override
  {
    EBSD_INDEX_OUT_OF_RANGE(index < getSize());
    m_Ptr[index] = std::stoi(token);
  }

protected:
  UInt8Parser(uint8_t* ptr, size_t size, const std::string& name, int index)
  : m_Ptr(ptr)
  {
    setManageMemory(true);
    setSize(size);
    setColumnName(name);
    setColumnIndex(index);
  }

private:
  uint8_t* m_Ptr;

public:
  UInt8Parser(const UInt8Parser&) = delete;            // Copy Constructor Not Implemented
  UInt8Parser(UInt8Parser&&) = delete;                 // Move Constructor Not Implemented
  UInt8Parser& operator=(const UInt8Parser&) = delete; // Copy Assignment Not Implemented
  UInt8Parser& operator=(UInt8Parser&&) = delete;      // Move Assignment Not Implemented
};

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
class Int32Parser : public DataParser
{
public:
  using Self = Int32Parser;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer()
  {
    return Pointer(static_cast<Self*>(nullptr));
  }

  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  std::string getNameOfClass() const
  {
    return std::string("Int32Parser");
  }
  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  static std::string ClassName()
  {
    return std::string("Int32Parser");
  }

  EbsdLib::NumericTypes::Type getNumericType() const override
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }

  int32_t IsA() const override
  {
    return 1;
  }

  static Pointer New(int32_t* ptr, size_t size, const std::string& name, int index)
  {
    Pointer sharedPtr(new Int32Parser(ptr, size, name, index));
    return sharedPtr;
  }

  ~Int32Parser() override
  {
    if(m_Ptr != nullptr && getManageMemory())
    {
      delete[] m_Ptr;
      m_Ptr = nullptr;
    }
  }

  void setPtr(int32_t* value)
  {
    this->m_Ptr = value;
  }
  int32_t* getPtr()
  {
    return m_Ptr;
  }

  bool allocateArray(size_t numberOfElements) override
  {
    m_Ptr = new(std::nothrow) int32_t[numberOfElements]();
    return (m_Ptr != nullptr);
  }

  void* getVoidPointer() override
  {
    return reinterpret_cast<void*>(m_Ptr);
  }
  void setVoidPointer(void* p) override
  {
    m_Ptr = reinterpret_cast<int32_t*>(p);
  }

  int32_t* getPointer(size_t offset)
  {
    return m_Ptr + offset;
  }

  void parse(const std::string& token, size_t index) override
  {
    EBSD_INDEX_OUT_OF_RANGE(index < getSize());
    m_Ptr[index] = std::stoi(token);
  }

protected:
  Int32Parser(int32_t* ptr, size_t size, const std::string& name, int index)
  : m_Ptr(ptr)
  {
    setManageMemory(true);
    setSize(size);
    setColumnName(name);
    setColumnIndex(index);
  }

private:
  int32_t* m_Ptr;

public:
  Int32Parser(const Int32Parser&) = delete;            // Copy Constructor Not Implemented
  Int32Parser(Int32Parser&&) = delete;                 // Move Constructor Not Implemented
  Int32Parser& operator=(const Int32Parser&) = delete; // Copy Assignment Not Implemented
  Int32Parser& operator=(Int32Parser&&) = delete;      // Move Assignment Not Implemented
};

// -----------------------------------------------------------------------------
class FloatParser : public DataParser
{
public:
  using Self = FloatParser;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer()
  {
    return Pointer(static_cast<Self*>(nullptr));
  }

  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  const std::string getNameOfClass() const
  {
    return std::string("FloatParser");
  }
  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  static std::string ClassName()
  {
    return std::string("FloatParser");
  }

  EbsdLib::NumericTypes::Type getNumericType() const override
  {
    return EbsdLib::NumericTypes::Type::Float;
  }

  int32_t IsA() const override
  {
    return 2;
  }

  static Pointer New(float* ptr, size_t size, const std::string& name, int index)
  {
    Pointer sharedPtr(new FloatParser(ptr, size, name, index));
    return sharedPtr;
  }

  ~FloatParser() override
  {
    if(m_Ptr != nullptr && getManageMemory())
    {
      delete[] m_Ptr;
      m_Ptr = nullptr;
    }
  }

  void setPtr(float* value)
  {
    this->m_Ptr = value;
  }
  float* getPtr()
  {
    return m_Ptr;
  }

  bool allocateArray(size_t numberOfElements) override
  {
    m_Ptr = new(std::nothrow) float[numberOfElements]();
    return (m_Ptr != nullptr);
  }

  void* getVoidPointer() override
  {
    return reinterpret_cast<void*>(m_Ptr);
  }
  void setVoidPointer(void* p) override
  {
    m_Ptr = reinterpret_cast<float*>(p);
  }

  float* getPointer(size_t offset)
  {
    return m_Ptr + offset;
  }

  void parse(const std::string& token, size_t index) override
  {
    m_Ptr[index] = std::stof(token);
  }

protected:
  FloatParser(float* ptr, size_t size, const std::string& name, int index)
  : m_Ptr(ptr)
  {
    setManageMemory(true);
    setSize(size);
    setColumnName(name);
    setColumnIndex(index);
  }

private:
  float* m_Ptr;

public:
  FloatParser(const FloatParser&) = delete;            // Copy Constructor Not Implemented
  FloatParser(FloatParser&&) = delete;                 // Move Constructor Not Implemented
  FloatParser& operator=(const FloatParser&) = delete; // Copy Assignment Not Implemented
  FloatParser& operator=(FloatParser&&) = delete;      // Move Assignment Not Implemented
};

template <typename T>
class NumericParser : public DataParser
{
public:
  using Self = NumericParser<T>;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer()
  {
    return Pointer(static_cast<Self*>(nullptr));
  }

  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  const std::string getNameOfClass() const
  {
    return std::string("NumericParser");
  }
  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  static std::string ClassName()
  {
    return std::string("NumericParser");
  }

  int32_t IsA() const override
  {
    return 2;
  }

  static Pointer New(T* ptr, size_t size, const std::string& name, int index)
  {
    Pointer sharedPtr(new NumericParser<T>(ptr, size, name, index));
    return sharedPtr;
  }

  ~NumericParser() override
  {
    if(m_Ptr != nullptr && getManageMemory())
    {
      delete[] m_Ptr;
      m_Ptr = nullptr;
    }
  }

  void setPtr(T* value)
  {
    this->m_Ptr = value;
  }
  T* getPtr()
  {
    return m_Ptr;
  }

  bool allocateArray(size_t numberOfElements) override
  {
    m_Ptr = new(std::nothrow) T[numberOfElements]();
    return (m_Ptr != nullptr);
  }

  void* getVoidPointer() override
  {
    return reinterpret_cast<void*>(m_Ptr);
  }
  void setVoidPointer(void* p) override
  {
    m_Ptr = reinterpret_cast<T*>(p);
  }

  T* getPointer(size_t offset)
  {
    return m_Ptr + offset;
  }

  void parse(const std::string& token, size_t index) override
  {
    m_Ptr[index] = std::stof(token);
  }

protected:
  NumericParser(T* ptr, size_t size, const std::string& name, int index)
  : m_Ptr(ptr)
  {
    setManageMemory(true);
    setSize(size);
    setColumnName(name);
    setColumnIndex(index);
  }

private:
  T* m_Ptr;

public:
  NumericParser(const NumericParser&) = delete;            // Copy Constructor Not Implemented
  NumericParser(NumericParser&&) = delete;                 // Move Constructor Not Implemented
  NumericParser& operator=(const NumericParser&) = delete; // Copy Assignment Not Implemented
  NumericParser& operator=(NumericParser&&) = delete;      // Move Assignment Not Implemented
};
