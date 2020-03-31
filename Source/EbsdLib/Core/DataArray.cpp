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
 * The code contained herein was partially funded by the followig contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "DataArray.hpp"

#include <QtCore/QDebug>

//  Allow user to force EBSDLIB_ENDIAN_NO_INTRINSICS in case they aren't available for a
//  particular platform/compiler combination.
#ifndef EBSDLIB_ENDIAN_NO_INTRINSICS

#ifndef __has_builtin      // Optional of course
#define __has_builtin(x) 0 // Compatibility with non-clang compilers
#endif

#if defined(_MSC_VER)
//  Microsoft documents these as being compatible since Windows 95 and specifically
//  lists runtime library support since Visual Studio 2003 (aka 7.1).
//  Clang/c2 uses the Microsoft rather than GCC intrinsics, so we check for
//  defined(_MSC_VER) before defined(__clang__)
#define EBSDLIB_ENDIAN_INTRINSIC_MSG "cstdlib _byteswap_ushort, etc."
#include <cstdlib>
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_2(x) _byteswap_ushort(x)
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_4(x) _byteswap_ulong(x)
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_8(x) _byteswap_uint64(x)

//  GCC and Clang recent versions provide intrinsic byte swaps via builtins
#elif(defined(__clang__) && __has_builtin(__builtin_bswap32) && __has_builtin(__builtin_bswap64)) || (defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 3)))
#define EBSDLIB_ENDIAN_INTRINSIC_MSG "__builtin_bswap16, etc."
// prior to 4.8, gcc did not provide __builtin_bswap16 on some platforms so we emulate it
// see http://gcc.gnu.org/bugzilla/show_bug.cgi?id=52624
// Clang has a similar problem, but their feature test macros make it easier to detect
#if(defined(__clang__) && __has_builtin(__builtin_bswap16)) || (defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 8)))
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_2(x) __builtin_bswap16(x)
#else
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_2(x) __builtin_bswap32((x) << 16)
#endif
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_4(x) __builtin_bswap32(x)
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_8(x) __builtin_bswap64(x)

//  Linux systems provide the byteswap.h header, with
#elif defined(__linux__)
//  don't check for obsolete forms defined(linux) and defined(__linux) on the theory that
//  compilers that predefine only these are so old that byteswap.h probably isn't present.
#define EBSDLIB_ENDIAN_INTRINSIC_MSG "byteswap.h bswap_16, etc."
#include <byteswap.h>
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_2(x) bswap_16(x)
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_4(x) bswap_32(x)
#define EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_8(x) bswap_64(x)

#else
#define EBSDLIB_ENDIAN_NO_INTRINSICS
#define EBSDLIB_ENDIAN_INTRINSIC_MSG "no byte swap intrinsics"
#endif

#elif !defined(EBSDLIB_ENDIAN_INTRINSIC_MSG)
#define EBSDLIB_ENDIAN_INTRINSIC_MSG "no byte swap intrinsics"
#endif // EBSDLIB_ENDIAN_NO_INTRINSICS

#include <string>
#include <cstring>
#include <functional>
#include <iostream>
#include <numeric>

#ifdef DATA_ARRAY_ENABLE_HDF5_IO
#include "SIMPLib/HDF5/H5DataArrayReader.h"
#include "SIMPLib/HDF5/H5DataArrayWriter.hpp"
#endif

#define DA_BYTE_SWAP(s, d, t)                                                                                                                                                                          \
  t[0] = ptr[s];                                                                                                                                                                                       \
  ptr[s] = ptr[d];                                                                                                                                                                                     \
  ptr[d] = t[0];

template <typename T>
typename DataArray<T>::Pointer DataArray<T>::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

template <typename T>
QString DataArray<T>::getNameOfClass() const
{
  return QString("DataArray<T>");
}

template <typename T>
QString DataArray<T>::ClassName()
{
  return QString("DataArray<T>");
}

template <typename T>
int32_t DataArray<T>::getClassVersion() const
{
  return 2;
}

//========================================= Constructing DataArray Objects =================================
template <typename T>
DataArray<T>::DataArray() = default;

/**
 * @brief Constructor
 * @param numTuples The number of Tuples in the DataArray
 * @param name The name of the DataArray
 * @param initValue The value to use when initializing each element of the array
 */
template <typename T>
DataArray<T>::DataArray(size_t numTuples, const QString& name, T initValue)
: m_Name(name)
, m_InitValue(initValue)
{
  resizeTuples(numTuples);
}

/**
 * @brief DataArray
 * @param numTuples The number of Tuples in the DataArray
 * @param name The name of the DataArray
 * @param compDims The number of elements in each axis dimension.
 * @param initValue The value to use when initializing each element of the array
 *
 * For example if you have a 2D image dimensions of 80(w) x 60(h) then the "cdims" would be [80][60]
 */
template <typename T>
DataArray<T>::DataArray(size_t numTuples, const QString& name, comp_dims_type compDims, T initValue)
: m_Name(name)
, m_NumTuples(numTuples)
, m_CompDims(std::move(compDims))
{
  m_NumComponents = std::accumulate(m_CompDims.begin(), m_CompDims.end(), static_cast<size_t>(1), std::multiplies<>());
  m_InitValue = initValue;
  m_Array = resizeAndExtend(m_NumTuples * m_NumComponents);
}

/**
 * @brief Protected Constructor
 * @param numTuples The number of Tuples in the DataArray
 * @param name The name of the DataArray
 * @param compDims The number of elements in each axis dimension.
 * @param initValue The value to use when initializing each element of the array
 * @param allocate Will all the memory be allocated at time of construction
 */
template <typename T>
DataArray<T>::DataArray(size_t numTuples, const QString& name, comp_dims_type compDims, T initValue, bool allocate)
: m_Name(name)
, m_NumTuples(numTuples)
, m_CompDims(std::move(compDims))
{
  m_NumComponents = std::accumulate(m_CompDims.begin(), m_CompDims.end(), static_cast<size_t>(1), std::multiplies<>());
  m_InitValue = initValue;
  if(allocate)
  {
    resizeTuples(numTuples);
  }
  else
  {
    m_Size = m_NumTuples * m_NumComponents;
    m_MaxId = (m_Size > 0) ? m_Size - 1 : m_Size;
  }
#if 0
      MUD_FLAP_0 = MUD_FLAP_1 = MUD_FLAP_2 = MUD_FLAP_3 = MUD_FLAP_4 = MUD_FLAP_5 = 0xABABABABABABABABul;
#endif
}

template <typename T>
DataArray<T>::~DataArray()
{
  clear();
}

template <typename T>
typename DataArray<T>::Pointer DataArray<T>::CreateArray(size_t numTuples, const QString& name, bool allocate)
{
  if(name.isEmpty())
  {
    return NullPointer();
  }
  comp_dims_type cDims = {1};
  auto d = new DataArray<T>(numTuples, name, cDims, static_cast<T>(0), allocate);
  if(allocate)
  {
    if(d->allocate() < 0)
    {
      // Could not allocate enough memory, reset the pointer to null and return
      delete d;
      return DataArray<T>::NullPointer();
    }
  }
  Pointer ptr(d);
  return ptr;
}

// -----------------------------------------------------------------------------
template <typename T>
typename DataArray<T>::Pointer DataArray<T>::CreateArray(size_t numTuples, int rank, const size_t* dims, const QString& name, bool allocate)
{
  if(name.isEmpty())
  {
    return NullPointer();
  }
  comp_dims_type cDims(static_cast<size_t>(rank));
  for(size_t i = 0; i < static_cast<size_t>(rank); i++)
  {
    cDims[i] = dims[i];
  }
  auto d = new DataArray<T>(numTuples, name, cDims, static_cast<T>(0), allocate);
  if(allocate)
  {
    if(d->allocate() < 0)
    {
      // Could not allocate enough memory, reset the pointer to null and return
      delete d;
      return DataArray<T>::NullPointer();
    }
  }
  Pointer ptr(d);
  return ptr;
}

// -----------------------------------------------------------------------------
template <typename T>
typename DataArray<T>::Pointer DataArray<T>::CreateArray(size_t numTuples, const comp_dims_type& compDims, const QString& name, bool allocate)
{
  if(name.isEmpty())
  {
    return NullPointer();
  }
  DataArray<T>* d = new DataArray<T>(numTuples, name, compDims, static_cast<T>(0), allocate);
  if(allocate)
  {
    if(d->allocate() < 0)
    {
      // Could not allocate enough memory, reset the pointer to null and return
      delete d;
      return DataArray<T>::NullPointer();
    }
  }
  Pointer ptr(d);
  return ptr;
}

// -----------------------------------------------------------------------------
template <typename T>
typename DataArray<T>::Pointer DataArray<T>::CreateArray(const comp_dims_type& tupleDims, const comp_dims_type& compDims, const QString& name, bool allocate)
{
  if(name.isEmpty())
  {
    return NullPointer();
  }

  size_t numTuples = std::accumulate(tupleDims.begin(), tupleDims.end(), static_cast<size_t>(1), std::multiplies<>());

  auto d = new DataArray<T>(numTuples, name, compDims, static_cast<T>(0), allocate);
  if(allocate)
  {
    if(d->allocate() < 0)
    {
      // Could not allocate enough memory, reset the pointer to null and return
      delete d;
      return DataArray<T>::NullPointer();
    }
  }
  Pointer ptr(d);
  return ptr;
}

template <typename T>
typename DataArray<T>::Pointer DataArray<T>::createNewArray(size_t numTuples, int rank, const size_t* compDims, const QString& name, bool allocate) const
{
  DataArray<T>::Pointer p = DataArray<T>::CreateArray(numTuples, rank, compDims, name, allocate);
  return p;
}

template <typename T>
typename DataArray<T>::Pointer DataArray<T>::createNewArray(size_t numTuples, const comp_dims_type& compDims, const QString& name, bool allocate) const
{
  DataArray<T>::Pointer p = DataArray<T>::CreateArray(numTuples, compDims, name, allocate);
  return p;
}

// -----------------------------------------------------------------------------
template <typename T>
typename DataArray<T>::Pointer DataArray<T>::FromQVector(QVector<T>& vec, const QString& name)
{

  Pointer p = CreateArray(static_cast<size_t>(vec.size()), name, true);
  if(nullptr != p.get())
  {
    std::memcpy(p->getPointer(0), vec.data(), static_cast<size_t>(vec.size()) * sizeof(T));
  }
  return p;
}

// -----------------------------------------------------------------------------
template <>
typename DataArray<bool>::Pointer DataArray<bool>::FromStdVector(std::vector<bool>& vec, const QString& name)
{
  comp_dims_type cDims = {1};
  Pointer p = CreateArray(vec.size(), cDims, name, true);
  if(nullptr != p.get())
  {
    size_t i = 0;
    for(const auto value : vec)
    {
      p->setValue(i, value);
      i++;
    }
  }
  return p;
}

// -----------------------------------------------------------------------------
template <typename T>
typename DataArray<T>::Pointer DataArray<T>::FromStdVector(std::vector<T>& vec, const QString& name)
{
  comp_dims_type cDims = {1};
  Pointer p = CreateArray(vec.size(), cDims, name, true);
  if(nullptr != p.get())
  {
    std::memcpy(p->getPointer(0), &(vec.front()), vec.size() * sizeof(T));
  }
  return p;
}

// -----------------------------------------------------------------------------
template <typename T>
typename DataArray<T>::Pointer DataArray<T>::CopyFromPointer(T* data, size_t size, const QString& name)
{
  Pointer p = CreateArray(size, name, true);
  if(nullptr != p.get())
  {
    std::memcpy(p->getPointer(0), data, size * sizeof(T));
  }
  return p;
}

/**
 * @brief WrapPointer Creates a DataArray<T> object that references the pointer. The original caller can
 * set if the memory should be "free()'ed" when the object goes away. The original memory MUST have been
 * "alloc()'ed" and <b>NOT</b> new 'ed.
 * @param data
 * @param numTuples
 * @param cDims
 * @param name
 * @param ownsData
 * @return
 */
template <typename T>
typename DataArray<T>::Pointer DataArray<T>::WrapPointer(T* data, size_t numTuples, const comp_dims_type& compDims, const QString& name, bool ownsData)
{
  // Allocate on the heap
  auto d = new DataArray(numTuples, name, compDims, static_cast<T>(0), false);
  // Wrap that heap pointer with a shared_pointer to make it reference counted
  Pointer p(d);

  p->m_Array = data;        // Now set the internal array to the raw pointer
  p->m_OwnsData = ownsData; // Set who owns the data, i.e., who is going to "free" the memory
  if(nullptr != data)
  {
    p->m_IsAllocated = true;
  }

  return p;
}

//========================================= Begin API =================================

template <typename T>
void DataArray<T>::setName(const QString& name)
{
  m_Name = name;
}

template <typename T>
QString DataArray<T>::getName() const
{
  return m_Name;
}

template <typename T>
typename DataArray<T>::Pointer DataArray<T>::deepCopy(bool forceNoAllocate) const
{
  bool allocate = m_IsAllocated;
  if(forceNoAllocate)
  {
    allocate = false;
  }
  DataArray<T>::Pointer daCopy = createNewArray(getNumberOfTuples(), getComponentDimensions(), getName(), allocate);
  if(m_IsAllocated && !forceNoAllocate)
  {
    T* src = getPointer(0);
    void* dest = daCopy->getVoidPointer(0);
    size_t totalBytes = (getNumberOfTuples() * static_cast<size_t>(getNumberOfComponents()) * sizeof(T));
    std::memcpy(dest, src, totalBytes);
  }
  return daCopy;
}

/**
 * @brief GetTypeName Returns a string representation of the type of data that is stored by this class. This
 * can be a primitive like char, float, int or the name of a class.
 * @return
 */
template <typename T>
EbsdLib::NumericTypes::Type DataArray<T>::getType() const
{
  T value = static_cast<T>(0x00);
  if(typeid(value) == typeid(int8_t))
  {
    return EbsdLib::NumericTypes::Type::Int8;
  }
  if(typeid(value) == typeid(uint8_t))
  {
    return EbsdLib::NumericTypes::Type::UInt8;
  }

  if(typeid(value) == typeid(int16_t))
  {
    return EbsdLib::NumericTypes::Type::Int16;
  }
  if(typeid(value) == typeid(uint16_t))
  {
    return EbsdLib::NumericTypes::Type::UInt16;
  }

  if(typeid(value) == typeid(int32_t))
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(typeid(value) == typeid(uint32_t))
  {
    return EbsdLib::NumericTypes::Type::UInt32;
  }

  if(typeid(value) == typeid(int64_t))
  {
    return EbsdLib::NumericTypes::Type::Int64;
  }
  if(typeid(value) == typeid(uint64_t))
  {
    return EbsdLib::NumericTypes::Type::UInt64;
  }

  if(typeid(value) == typeid(float))
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(typeid(value) == typeid(double))
  {
    return EbsdLib::NumericTypes::Type::Double;
  }

  if(typeid(value) == typeid(bool))
  {
    return EbsdLib::NumericTypes::Type::Bool;
  }

  return EbsdLib::NumericTypes::Type::UnknownNumType;
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::getXdmfTypeAndSize(QString& xdmfTypeName, int& precision) const
{
  T value = static_cast<T>(0x00);
  xdmfTypeName = "UNKNOWN";
  precision = 0;
  if(typeid(value) == typeid(int8_t))
  {
    xdmfTypeName = "Char";
    precision = 1;
  }
  if(typeid(value) == typeid(uint8_t))
  {
    xdmfTypeName = "UChar";
    precision = 1;
  }

  if(typeid(value) == typeid(int16_t))
  {
    xdmfTypeName = "Int";
    precision = 2;
  }
  if(typeid(value) == typeid(uint16_t))
  {
    xdmfTypeName = "UInt";
    precision = 2;
  }

  if(typeid(value) == typeid(int32_t))
  {
    xdmfTypeName = "Int";
    precision = 4;
  }
  if(typeid(value) == typeid(uint32_t))
  {
    xdmfTypeName = "UInt";
    precision = 4;
  }

  if(typeid(value) == typeid(int64_t))
  {
    xdmfTypeName = "Int";
    precision = 8;
  }
  if(typeid(value) == typeid(uint64_t))
  {
    xdmfTypeName = "UInt";
    precision = 8;
  }

  if(typeid(value) == typeid(float))
  {
    xdmfTypeName = "Float";
    precision = 4;
  }
  if(typeid(value) == typeid(double))
  {
    xdmfTypeName = "Float";
    precision = 8;
  }

  if(typeid(value) == typeid(bool))
  {
    xdmfTypeName = "uchar";
    precision = 1;
  }
}
// This line must be here, because we are overloading the copyData pure  function in DataArray<T>.
// This is required so that other classes can call this version of copyData from the subclasses.
// using DataArray<T>::copyFromArray;

// -----------------------------------------------------------------------------
template <typename T>
bool DataArray<T>::copyFromArray(size_t destTupleOffset, DataArray<T>::Pointer sourceArray, size_t srcTupleOffset, size_t totalSrcTuples)
{
  if(!m_IsAllocated)
  {
    return false;
  }
  if(nullptr == m_Array)
  {
    return false;
  }
  if(destTupleOffset > m_MaxId)
  {
    return false;
  }
  if(!sourceArray->isAllocated())
  {
    return false;
  }
  Self* source = dynamic_cast<Self*>(sourceArray.get());
  if(nullptr == source->getPointer(0))
  {
    return false;
  }

  if(sourceArray->getNumberOfComponents() != getNumberOfComponents())
  {
    return false;
  }

  if(srcTupleOffset + totalSrcTuples > sourceArray->getNumberOfTuples())
  {
    return false;
  }

  if(totalSrcTuples * static_cast<size_t>(sourceArray->getNumberOfComponents()) + destTupleOffset * static_cast<size_t>(getNumberOfComponents()) > m_Size)
  {
    return false;
  }

  size_t elementStart = destTupleOffset * static_cast<size_t>(getNumberOfComponents());
  size_t totalBytes = (totalSrcTuples * static_cast<size_t>(sourceArray->getNumberOfComponents())) * sizeof(T);
  std::memcpy(m_Array + elementStart, source->getPointer(srcTupleOffset * static_cast<size_t>(sourceArray->getNumberOfComponents())), totalBytes);
  return true;
}

// -----------------------------------------------------------------------------
template <typename T>
bool DataArray<T>::copyIntoArray(Pointer dest) const
{
  if(m_IsAllocated && dest->isAllocated() && m_Array && dest->getPointer(0))
  {
    size_t totalBytes = m_Size * sizeof(T);
    std::memcpy(dest->getPointer(0), m_Array, totalBytes);
    return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
template <typename T>
bool DataArray<T>::isAllocated() const
{
  return m_IsAllocated;
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::setInitValue(T initValue)
{
  m_InitValue = initValue;
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::takeOwnership()
{
  m_OwnsData = true;
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::releaseOwnership()
{
  m_OwnsData = false;
}

// -----------------------------------------------------------------------------
template <typename T>
int32_t DataArray<T>::allocate()
{
  if((nullptr != m_Array) && (true == m_OwnsData))
  {
    deallocate();
  }
  m_Array = nullptr;
  m_OwnsData = true;
  m_IsAllocated = false;
  if(m_Size == 0)
  {
    clear();
    return 1;
  }

  size_t newSize = m_Size;
  m_Array = new T[newSize]();
  if(!m_Array)
  {
    qDebug() << "Unable to allocate " << newSize << " elements of size " << sizeof(T) << " bytes. ";
    return -1;
  }
  m_Size = newSize;
  m_IsAllocated = true;

  return 1;
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::initializeWithZeros()
{
  if(!m_IsAllocated || nullptr == m_Array)
  {
    return;
  }
  size_t typeSize = sizeof(T);
  ::memset(m_Array, 0, m_Size * typeSize);
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::initializeWithValue(T initValue, size_t offset)
{
  if(!m_IsAllocated || nullptr == m_Array)
  {
    return;
  }
  for(size_t i = offset; i < m_Size; i++)
  {
    m_Array[i] = initValue;
  }
}

// -----------------------------------------------------------------------------
template <typename T>
int DataArray<T>::eraseTuples(comp_dims_type& idxs)
{
  int err = 0;

  // If nothing is to be erased just return
  if(idxs.empty())
  {
    return 0;
  }
  auto idxs_size = static_cast<size_t>(idxs.size());
  if(idxs_size >= getNumberOfTuples())
  {
    resizeTuples(0);
    return 0;
  }

  // Sanity Check the Indices in the vector to make sure we are not trying to remove any indices that are
  // off the end of the array and return an error code.
  for(size_t& idx : idxs)
  {
    if(idx * m_NumComponents > m_MaxId)
    {
      return -100;
    }
  }

  // Calculate the new size of the array to copy into
  size_t newSize = (getNumberOfTuples() - idxs.size()) * m_NumComponents;

  // Create a new m_Array to copy into
  T* newArray = new T[newSize]();

  // Splat AB across the array so we know if we are copying the values or not
  ::memset(newArray, 0xAB, newSize * sizeof(T));

  // Keep the current Destination Pointer
  T* currentDest = newArray;
  size_t j = 0;
  size_t k = 0;
  // Find the first chunk to copy by walking the idxs array until we get an
  // index that is NOT a continuous increment from the start
  for(k = 0; k < idxs.size(); ++k)
  {
    if(j == idxs[k])
    {
      ++j;
    }
    else
    {
      break;
    }
  }

  if(k == idxs.size()) // Only front elements are being dropped
  {
    T* currentSrc = m_Array + (j * m_NumComponents);
    std::memcpy(currentDest, currentSrc, (getNumberOfTuples() - idxs.size()) * m_NumComponents * sizeof(T));
    deallocate(); // We are done copying - delete the current m_Array
    m_Size = newSize;
    m_Array = newArray;
    m_OwnsData = true;
    m_MaxId = newSize - 1;
    m_IsAllocated = true;
    return 0;
  }

  comp_dims_type srcIdx(idxs.size() + 1);
  comp_dims_type destIdx(idxs.size() + 1);
  comp_dims_type copyElements(idxs.size() + 1);
  srcIdx[0] = 0;
  destIdx[0] = 0;
  copyElements[0] = (idxs[0] - 0) * m_NumComponents;

  for(size_t i = 1; i < srcIdx.size(); ++i)
  {
    srcIdx[i] = (idxs[i - 1] + 1) * m_NumComponents;

    if(i < srcIdx.size() - 1)
    {
      copyElements[i] = (idxs[i] - idxs[i - 1] - 1) * m_NumComponents;
    }
    else
    {
      copyElements[i] = (getNumberOfTuples() - idxs[i - 1] - 1) * m_NumComponents;
    }
    destIdx[i] = copyElements[i - 1] + destIdx[i - 1];
  }

  // Copy the data
  for(size_t i = 0; i < srcIdx.size(); ++i)
  {
    currentDest = newArray + destIdx[i];
    T* currentSrc = m_Array + srcIdx[i];
    size_t bytes = copyElements[i] * sizeof(T);
    std::memcpy(currentDest, currentSrc, bytes);
  }

  // We are done copying - delete the current m_Array
  deallocate();

  // Allocation was successful.  Save it.
  m_Size = newSize;
  m_Array = newArray;
  // This object has now allocated its memory and owns it.
  m_OwnsData = true;
  m_IsAllocated = true;
  m_MaxId = newSize - 1;

  return err;
}

// -----------------------------------------------------------------------------
template <typename T>
int DataArray<T>::copyTuple(size_t currentPos, size_t newPos)
{
  size_t max = ((m_MaxId + 1) / m_NumComponents);
  if(currentPos >= max || newPos >= max)
  {
    return -1;
  }
  T* src = m_Array + (currentPos * m_NumComponents);
  T* dest = m_Array + (newPos * m_NumComponents);
  size_t bytes = sizeof(T) * m_NumComponents;
  std::memcpy(dest, src, bytes);
  return 0;
}

// -----------------------------------------------------------------------------
template <typename T>
size_t DataArray<T>::getTypeSize() const
{
  return sizeof(T);
}

// -----------------------------------------------------------------------------
template <typename T>
size_t DataArray<T>::getNumberOfTuples() const
{
  return m_NumTuples;
}

// -----------------------------------------------------------------------------
template <typename T>
size_t DataArray<T>::getSize() const
{
  return m_Size;
}

// -----------------------------------------------------------------------------
template <typename T>
typename DataArray<T>::comp_dims_type DataArray<T>::getComponentDimensions() const
{
  return m_CompDims;
}

// -----------------------------------------------------------------------------
template <typename T>
int32_t DataArray<T>::getNumberOfComponents() const
{
  return static_cast<int32_t>(m_NumComponents);
}

// -----------------------------------------------------------------------------
template <typename T>
void* DataArray<T>::getVoidPointer(size_t i)
{
  if(i >= m_Size)
  {
    return nullptr;
  }

  return reinterpret_cast<void*>(&(m_Array[i]));
}

// -----------------------------------------------------------------------------
template <typename T>
std::list<T> DataArray<T>::getArray() const
{
  return std::list<T>(m_Array, m_Array + (m_Size * sizeof(T)) / sizeof(T));
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::setArray(std::list<T> newArray)
{
  if(newArray.size() != m_Size)
  {
    return;
  }
  int i = 0;
  for(auto elem : newArray)
  {
    m_Array[i++] = elem;
  }
}

// -----------------------------------------------------------------------------
template <typename T>
T* DataArray<T>::getPointer(size_t i) const
{
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(i < m_Size);
  }
#endif
  return m_Array + i;
}

// -----------------------------------------------------------------------------
template <typename T>
T DataArray<T>::getValue(size_t i) const
{
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(i < m_Size);
  }
#endif
  return m_Array[i];
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::setValue(size_t i, T value)
{
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(i < m_Size);
  }
#endif
  m_Array[i] = value;
}

// -----------------------------------------------------------------------------
template <typename T>
T DataArray<T>::getComponent(size_t i, int32_t j) const
{
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(i * m_NumComponents + static_cast<size_t>(j) < m_Size);
  }
#endif
  return m_Array[i * m_NumComponents + static_cast<size_t>(j)];
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::setComponent(size_t i, int32_t j, T c)
{
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(i * m_NumComponents + static_cast<size_t>(j) < m_Size);
  }
#endif
  m_Array[i * m_NumComponents + static_cast<size_t>(j)] = c;
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::setTuple(size_t tupleIndex, T* data)
{
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(tupleIndex * m_NumComponents + (m_NumComponents - 1) < m_Size);
  }
#endif
  std::memcpy(getTuplePointer(tupleIndex), data, m_NumComponents * sizeof(T));
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::setTuple(size_t tupleIndex, const std::vector<T>& data)
{
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(tupleIndex * m_NumComponents + (m_NumComponents - 1) < m_Size);
  }
#endif
  std::memcpy(getTuplePointer(tupleIndex), data.data(), m_NumComponents * sizeof(T));
}
// -----------------------------------------------------------------------------
template <>
void DataArray<bool>::setTuple(size_t tupleIndex, const std::vector<bool>& data)
{
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(tupleIndex * m_NumComponents + (m_NumComponents - 1) < m_Size);
  }
#endif
  bool* ptr = getTuplePointer(tupleIndex);
  size_t i = 0;
  for(const auto value : data)
  {
    ptr[i] = value;
    i++;
  }
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::initializeTuple(size_t i, void* p)
{
  if(!m_IsAllocated)
  {
    return;
  }
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(i * m_NumComponents < m_Size);
  }
#endif
  if(nullptr == p)
  {
    return;
  }
  T* c = reinterpret_cast<T*>(p);
  for(size_t j = 0; j < m_NumComponents; ++j)
  {
    m_Array[i * m_NumComponents + j] = *c;
  }
}

// -----------------------------------------------------------------------------
template <typename T>
T* DataArray<T>::getTuplePointer(size_t tupleIndex) const
{
#ifndef NDEBUG
  if(m_Size > 0)
  {
    Q_ASSERT(tupleIndex * m_NumComponents < m_Size);
  }
#endif
  return m_Array + (tupleIndex * m_NumComponents);
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::resizeTuples(size_t numTuples)
{
  T* ptr = resizeAndExtend(numTuples * m_NumComponents);
  if(nullptr != ptr)
  {
    m_NumTuples = numTuples;
  }
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::printTuple(QTextStream& out, size_t i, char delimiter) const
{
  int precision = out.realNumberPrecision();
  T value = static_cast<T>(0x00);
  if(typeid(value) == typeid(float))
  {
    out.setRealNumberPrecision(8);
  }
  if(typeid(value) == typeid(double))
  {
    out.setRealNumberPrecision(16);
  }

  for(size_t j = 0; j < m_NumComponents; ++j)
  {
    if(j != 0)
    {
      out << delimiter;
    }
    out << m_Array[i * m_NumComponents + j];
  }
  out.setRealNumberPrecision(precision);
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::printComponent(QTextStream& out, size_t i, int32_t j) const
{
  out << m_Array[i * m_NumComponents + static_cast<size_t>(j)];
}

// -----------------------------------------------------------------------------
template <typename T>
QString DataArray<T>::getFullNameOfClass() const
{
  QString theType = getTypeAsString();
  theType = "DataArray<" + theType + ">";
  return theType;
}

// -----------------------------------------------------------------------------
template <typename T>
QString DataArray<T>::getTypeAsString() const
{
  T value = static_cast<T>(0);
  if(typeid(value) == typeid(float))
  {
    return "float";
  }
  if(typeid(value) == typeid(double))
  {
    return "double";
  }

  if(typeid(value) == typeid(int8_t))
  {
    return "int8_t";
  }
  if(typeid(value) == typeid(uint8_t))
  {
    return "uint8_t";
  }
#if CMP_TYPE_CHAR_IS_SIGNED
  if(typeid(value) == typeid(char))
  {
    return "int8_t";
  }
#else
  if(typeid(value) == typeid(char))
  {
    return "int8_t";
  }
#endif
  if(typeid(value) == typeid(signed char))
  {
    return "int8_t";
  }
  if(typeid(value) == typeid(unsigned char))
  {
    return "uint8_t";
  }

  if(typeid(value) == typeid(int16_t))
  {
    return "int16_t";
  }
  if(typeid(value) == typeid(short))
  {
    return "int16_t";
  }
  if(typeid(value) == typeid(signed short))
  {
    return "int16_t";
  }
  if(typeid(value) == typeid(uint16_t))
  {
    return "uint16_t";
  }
  if(typeid(value) == typeid(unsigned short))
  {
    return "uint16_t";
  }

  if(typeid(value) == typeid(int32_t))
  {
    return "int32_t";
  }
  if(typeid(value) == typeid(uint32_t))
  {
    return "uint32_t";
  }
#if(CMP_SIZEOF_INT == 4)
  if(typeid(value) == typeid(int))
  {
    return "int32_t";
  }
  if(typeid(value) == typeid(signed int))
  {
    return "int32_t";
  }
  if(typeid(value) == typeid(unsigned int))
  {
    return "uint32_t";
  }
#endif

  if(typeid(value) == typeid(int64_t))
  {
    return "int64_t";
  }
  if(typeid(value) == typeid(uint64_t))
  {
    return "uint64_t";
  }

#if(CMP_SIZEOF_LONG == 4)
  if(typeid(value) == typeid(long int))
  {
    return "long int";
  }
  if(typeid(value) == typeid(signed long int))
  {
    return "signed long int";
  }
  if(typeid(value) == typeid(unsigned long int))
  {
    return "unsigned long int";
  }
#elif(CMP_SIZEOF_LONG == 8)
  if(typeid(value) == typeid(long int))
  {
    return "int64_t";
  }
  if(typeid(value) == typeid(signed long int))
  {
    return "int64_t";
  }
  if(typeid(value) == typeid(unsigned long int))
  {
    return "uint64_t";
  }
#endif

#if(CMP_SIZEOF_LONG_LONG == 8)
  if(typeid(value) == typeid(long long int))
  {
    return "int64_t";
  }
  if(typeid(value) == typeid(signed long long int))
  {
    return "int64_t";
  }
  if(typeid(value) == typeid(unsigned long long int))
  {
    return "uint64_t";
  }
#endif

  if(typeid(value) == typeid(bool))
  {
    return "bool";
  }

  // qDebug()  << "Error: HDFTypeForPrimitive - Unknown Type: " << (typeid(value).name()) ;
  const char* name = typeid(value).name();
  if(nullptr != name && name[0] == 'l')
  {
    qDebug() << "You are using 'long int' as a type which is not 32/64 bit safe. Suggest you use one of the H5SupportTypes defined in <Common/H5SupportTypes.h> such as int32_t or uint32_t.";
  }
  return "UnknownType";
}

#ifdef DATA_ARRAY_ENABLE_HDF5_IO
// -----------------------------------------------------------------------------
template <typename T>
int DataArray<T>::writeH5Data(hid_t parentId, comp_dims_type tDims) const
{
  if(m_Array == nullptr)
  {
    return -85648;
  }
  return H5DataArrayWriter::writeDataArray<Self>(parentId, this, tDims);
}
#endif
// -----------------------------------------------------------------------------
template <typename T>
int DataArray<T>::writeXdmfAttribute(QTextStream& out, int64_t* volDims, const QString& hdfFileName, const QString& groupPath, const QString& label) const
{
  if(m_Array == nullptr)
  {
    return -85648;
  }
  QString dimStr;
  int precision = 0;
  QString xdmfTypeName;
  getXdmfTypeAndSize(xdmfTypeName, precision);
  if(0 == precision)
  {
    out << "<!-- " << getName() << " has unknown type or unsupported type or precision for XDMF to understand"
        << " -->"
        << "\n";
    return -100;
  }

  int numComp = getNumberOfComponents();
  out << "    <Attribute Name=\"" << getName() << label << "\" ";
  if(numComp == 1)
  {
    out << "AttributeType=\"Scalar\" ";
    dimStr = QString("%1 %2 %3 ").arg(volDims[2]).arg(volDims[1]).arg(volDims[0]);
  }
  else if(numComp == 6)
  {
    out << "AttributeType=\"Tensor6\" ";
    dimStr = QString("%1 %2 %3 %4 ").arg(volDims[2]).arg(volDims[1]).arg(volDims[0]).arg(numComp);
  }
  else if(numComp == 9)
  {
    out << "AttributeType=\"Tensor\" ";
    dimStr = QString("%1 %2 %3 %4 ").arg(volDims[2]).arg(volDims[1]).arg(volDims[0]).arg(numComp);
  }
  else
  {
    out << "AttributeType=\"Vector\" ";
    dimStr = QString("%1 %2 %3 %4 ").arg(volDims[2]).arg(volDims[1]).arg(volDims[0]).arg(numComp);
  }
  out << "Center=\"Cell\">\n";
  // Open the <DataItem> Tag
  out << R"(      <DataItem Format="HDF" Dimensions=")" << dimStr << R"(" )";
  out << "NumberType=\"" << xdmfTypeName << "\" "
      << "Precision=\"" << precision << "\" >\n";

  out << "        " << hdfFileName << groupPath << "/" << getName() << "\n";
  out << "      </DataItem>"
      << "\n";
  out << "    </Attribute>"
      << "\n";
  return 1;
}

// -----------------------------------------------------------------------------
template <typename T>
QString DataArray<T>::getInfoString(EbsdLib::InfoStringFormat format) const
{

  QLocale usa(QLocale::English, QLocale::UnitedStates);

  QString info;
  QTextStream ss(&info);
  if(format == EbsdLib::HtmlFormat)
  {
    ss << "<html><head></head>\n";
    ss << "<body>\n";
    ss << "<table cellpadding=\"4\" cellspacing=\"0\" border=\"0\">\n";
    ss << "<tbody>\n";
    ss << "<tr bgcolor=\"#FFFCEA\"><th colspan=2>Attribute Array Info</th></tr>";

    ss << R"(<tr bgcolor="#E9E7D6"><th align="right">Name:</th><td>)" << getName() << "</td></tr>";

    ss << R"(<tr bgcolor="#FFFCEA"><th align="right">Type:</th><td> DataArray&lt;)" << getTypeAsString() << "&gt;</td></tr>";
    QString numStr = usa.toString(static_cast<qlonglong>(getNumberOfTuples()));
    ss << R"(<tr bgcolor="#FFFCEA"><th align="right">Number of Tuples:</th><td>)" << numStr << "</td></tr>";

    QString compDimStr = "(";
    for(size_t i = 0; i < m_CompDims.size(); i++)
    {
      compDimStr = compDimStr + QString::number(m_CompDims[i]);
      if(i < m_CompDims.size() - 1)
      {
        compDimStr = compDimStr + QString(", ");
      }
    }
    compDimStr = compDimStr + ")";
    ss << R"(<tr bgcolor="#FFFCEA"><th align="right">Component Dimensions:</th><td>)" << compDimStr << "</td></tr>";
    numStr = usa.toString(static_cast<qlonglong>(m_Size));
    ss << R"(<tr bgcolor="#FFFCEA"><th align="right">Total Elements:</th><td>)" << numStr << "</td></tr>";
    numStr = usa.toString(static_cast<qlonglong>(m_Size * sizeof(T)));
    ss << R"(<tr bgcolor="#FFFCEA"><th align="right">Total Memory Required:</th><td>)" << numStr << "</td></tr>";
    ss << "</tbody></table>\n";
    ss << "</body></html>";
  }
  else if(format == EbsdLib::MarkDown)
  {
    ss << "+ Name: " << getName() << "\n";
    ss << "+ Type: " << getTypeAsString() << "\n";
    ss << "+ Num. Tuple: " << getNumberOfTuples() << "\n";
    QString compDimStr = "(";
    for(size_t i = 0; i < m_CompDims.size(); i++)
    {
      compDimStr = compDimStr + QString::number(m_CompDims[i]);
      if(i < m_CompDims.size() - 1)
      {
        compDimStr = compDimStr + QString(", ");
      }
    }
    compDimStr = compDimStr + ")";
    ss << "+ Comp. Dims: " << compDimStr << "\n";
    ss << "+ Total Elements:  " << m_Size << "\n";
    ss << "+ Total Memory: " << (m_Size * sizeof(T)) << "\n";
  }
  return info;
}

#ifdef EbsdLib_ENABLE_HDF5
// -----------------------------------------------------------------------------
template <typename T>
int DataArray<T>::readH5Data(hid_t parentId)
{
  int err = -1;
#if 0
  resizeTuples(0);
  Pointer p = H5DataArrayReader::ReadIDataArray(parentId, getName());
  if(p.get() == nullptr)
  {
    return -1;
  }
  m_Array = reinterpret_cast<T*>(p->getVoidPointer(0));
  m_Size = p->getSize();
  m_OwnsData = true;
  m_MaxId = (m_Size == 0) ? 0 : m_Size - 1;
  m_IsAllocated = true;
  setName(p->getName());
  m_NumTuples = p->getNumberOfTuples();
  m_CompDims = p->getComponentDimensions();
  m_NumComponents = p->getNumberOfComponents();

  // Tell the intermediate DataArray to release ownership of the data as we are going to be responsible
  // for deleting the memory
  p->releaseOwnership();
#endif
  return err;
}

#endif

#if 0
// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::byteSwapElements()
{
  char* ptr = reinterpret_cast<char*>(m_Array);
  char t[8];
  size_t size = getTypeSize();
  for(uint64_t var = 0; var < m_Size; ++var)
  {
    if(sizeof(T) == 2)
    {
      DA_BYTE_SWAP(0, 1, t)
    }
    else if(sizeof(T) == 4)
    {
      DA_BYTE_SWAP(0, 3, t) DA_BYTE_SWAP(1, 2, t)
    }
    else if(sizeof(T) == 8)
    {
      DA_BYTE_SWAP(0, 7, t) DA_BYTE_SWAP(1, 6, t) DA_BYTE_SWAP(2, 5, t) DA_BYTE_SWAP(3, 4, t)
    }
    ptr += size; // increment the pointer
  }
}

#else
// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::byteSwapElements()
{
  for(size_t i = 0; i < m_Size; ++i)
  {
    if(sizeof(T) == 2)
    {
      m_Array[i] = EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_2(m_Array[i]);
    }
    else if(sizeof(T) == 4)
    {
      uint32_t* ptr = reinterpret_cast<uint32_t*>(m_Array + i);
      *ptr = EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_4(*ptr);
    }
    else if(sizeof(T) == 8)
    {
      uint64_t* ptr = reinterpret_cast<uint64_t*>(m_Array + i);
      *ptr = EBSDLIB_ENDIAN_INTRINSIC_BYTE_SWAP_8(*ptr);
    }
  }
}

#endif

template <typename T>
typename DataArray<T>::iterator DataArray<T>::begin()
{
  return iterator(m_Array);
}

template <typename T>
typename DataArray<T>::iterator DataArray<T>::end()
{
  return iterator(m_Array + m_Size);
}

template <typename T>
typename DataArray<T>::const_iterator DataArray<T>::begin() const
{
  return const_iterator(m_Array);
}
template <typename T>
typename DataArray<T>::const_iterator DataArray<T>::end() const
{
  return const_iterator(m_Array + m_Size);
}

// rbegin
// rend
// cbegin
// cend
// crbegin
// crend

// ######### Capacity #########
template <typename T>
typename DataArray<T>::size_type DataArray<T>::size() const
{
  return m_Size;
}
template <typename T>
typename DataArray<T>::size_type DataArray<T>::max_size() const
{
  return m_Size;
}
//  void resize(size_type n)
//  {
//    resizeAndExtend(n);
//  }
// void resize (size_type n, const value_type& val);
template <typename T>
typename DataArray<T>::size_type DataArray<T>::capacity() const noexcept
{
  return m_Size;
}

template <typename T>
bool DataArray<T>::empty() const noexcept
{
  return (m_Size == 0);
}
// reserve()
// shrink_to_fit()

// ######### Element Access #########

// ######### Modifiers #########

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::assign(size_type n, const value_type& val) // fill (2)
{
  resizeAndExtend(n);
  for(size_t i = 0; i < n; i++)
  {
    m_Array[i] = val;
  }
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::assign(std::initializer_list<value_type> il) //  initializer list (3)
{
  assign(il.begin(), il.end());
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::push_back(const value_type& val)
{
  resizeAndExtend(m_Size + 1);
  m_Array[m_MaxId] = val;
}
// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::push_back(value_type&& val)
{
  resizeAndExtend(m_Size + 1);
  m_Array[m_MaxId] = val;
}

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::pop_back()
{
  resizeAndExtend(m_Size - 1);
}
// insert
// iterator erase (const_iterator position)
// iterator erase (const_iterator first, const_iterator last);
// swap

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::clear()
{
  if(nullptr != m_Array && m_OwnsData)
  {
    deallocate();
  }
  m_Array = nullptr;
  m_Size = 0;
  m_OwnsData = true;
  m_MaxId = 0;
  m_IsAllocated = false;
  m_NumTuples = 0;
}
// emplace
// emplace_back

// =================================== END STL COMPATIBLE INTERFACe ===================================================

// -----------------------------------------------------------------------------
template <typename T>
void DataArray<T>::deallocate()
{
  // We are going to splat 0xABABAB across the first value of the array as a debugging aid
  auto cptr = reinterpret_cast<unsigned char*>(m_Array);
  if(nullptr != cptr)
  {
    if(m_Size > 0)
    {
      if(sizeof(T) >= 1)
      {
        cptr[0] = 0xAB;
      }
      if(sizeof(T) >= 2)
      {
        cptr[1] = 0xAB;
      }
      if(sizeof(T) >= 4)
      {
        cptr[2] = 0xAB;
        cptr[3] = 0xAB;
      }
      if(sizeof(T) >= 8)
      {
        cptr[4] = 0xAB;
        cptr[5] = 0xAB;
        cptr[6] = 0xAB;
        cptr[7] = 0xAB;
      }
    }
  }
#if 0
      if (MUD_FLAP_0 != 0xABABABABABABABABul
          || MUD_FLAP_1 != 0xABABABABABABABABul
          || MUD_FLAP_2 != 0xABABABABABABABABul
          || MUD_FLAP_3 != 0xABABABABABABABABul
          || MUD_FLAP_4 != 0xABABABABABABABABul
          || MUD_FLAP_5 != 0xABABABABABABABABul)
      {
        Q_ASSERT(false);
      }
#endif
  delete[](m_Array);

  m_Array = nullptr;
  m_IsAllocated = false;
}

// -----------------------------------------------------------------------------
template <typename T>
int32_t DataArray<T>::resizeTotalElements(size_t size)
{
  // std::cout << "DataArray::resizeTotalElements(" << size << ")" << std::endl;
  if(size == 0)
  {
    clear();
    return 1;
  }
  T* ptr = resizeAndExtend(size);
  if(nullptr != ptr)
  {
    return 1;
  }
  return 0;
}

// -----------------------------------------------------------------------------
template <typename T>
T* DataArray<T>::resizeAndExtend(size_t size)
{
  T* newArray = nullptr;
  size_t newSize = 0;
  size_t oldSize = 0;

  if(size == m_Size) // Requested size is equal to current size.  Do nothing.
  {
    return m_Array;
  }
  newSize = size;
  oldSize = m_Size;

  // Wipe out the array completely if new size is zero.
  if(newSize == 0)
  {
    clear();
    return m_Array;
  }

  newArray = new T[newSize]();
  if(!newArray)
  {
    qDebug() << "Unable to allocate " << newSize << " elements of size " << sizeof(T) << " bytes. ";
    return nullptr;
  }

  // Copy the data from the old array.
  if(m_Array != nullptr)
  {
    std::memcpy(newArray, m_Array, (newSize < m_Size ? newSize : m_Size) * sizeof(T));
  }

  // Allocate a new array if we DO NOT own the current array
  if((nullptr != m_Array) && m_OwnsData)
  {
    // Free the old array
    deallocate();
  }

  // Allocation was successful.  Save it.
  m_Size = newSize;
  m_Array = newArray;

  // This object has now allocated its memory and owns it.
  m_OwnsData = true;

  m_MaxId = newSize - 1;
  m_IsAllocated = true;

  // Initialize the new tuples if newSize is larger than old size
  if(newSize > oldSize)
  {
    initializeWithValue(m_InitValue, oldSize);
  }

  return m_Array;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
#if !defined(__APPLE__) && !defined(_MSC_VER)
#undef EbsdLib_EXPORT
#define EbsdLib_EXPORT
#endif

template class EbsdLib_EXPORT DataArray<bool>;

template class EbsdLib_EXPORT DataArray<char>;

template class EbsdLib_EXPORT DataArray<int8_t>;
template class EbsdLib_EXPORT DataArray<uint8_t>;

template class EbsdLib_EXPORT DataArray<int16_t>;
template class EbsdLib_EXPORT DataArray<uint16_t>;

template class EbsdLib_EXPORT DataArray<int32_t>;
template class EbsdLib_EXPORT DataArray<uint32_t>;

template class EbsdLib_EXPORT DataArray<int64_t>;
template class EbsdLib_EXPORT DataArray<uint64_t>;

template class EbsdLib_EXPORT DataArray<float>;
template class EbsdLib_EXPORT DataArray<double>;

#if defined(__APPLE__) || defined(_MSC_VER)
template class EbsdLib_EXPORT DataArray<size_t>;
#endif
