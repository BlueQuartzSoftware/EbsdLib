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

#pragma once

#include <cassert>
#include <cstring>
#include <functional>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <QtCore/QTextStream>
#include <QtCore/QDebug>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Core/EbsdLibConstants.h"

#ifdef EbsdLib_ENABLE_HDF5
#include <hdf5.h>
#endif

#define mxa_bswap(s, d, t)                                                                                                                                                                             \
  t[0] = ptr[s];                                                                                                                                                                                       \
  ptr[s] = ptr[d];                                                                                                                                                                                     \
  ptr[d] = t[0];

/**
 * @class DataArray DataArray.hpp DREAM3DLib/Common/DataArray.hpp
 * @brief Template class for wrapping raw arrays of data.
 * @author mjackson
 * @date July 3, 2008
 * @version $Revision: 1.2 $
 */
template <typename T>
class DataArray
{

public:
  using Self = DataArray<T>;
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
  const QString getNameOfClass() const
  {
    return QString("DataArray<T>");
  }
  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  static QString ClassName()
  {
    return QString("DataArray<T>");
  }

  /**
   * @brief Returns the version of this class.
   * @return
   */
  int32_t getClassVersion()
  {
    return 2;
  }

  DataArray(const DataArray&) = default;           // Copy Constructor default Implemented
  DataArray(DataArray&&) = delete;                 // Move Constructor Not Implemented
  DataArray& operator=(const DataArray&) = delete; // Copy Assignment Not Implemented
  DataArray& operator=(DataArray&&) = delete;      // Move Assignment Not Implemented

  //========================================= STL INTERFACE COMPATIBILITY =================================
  using comp_dims_type = std::vector<size_t>;
  using size_type = size_t;
  using value_type = T;
  using reference = T&;
  using iterator_category = std::input_iterator_tag;
  using pointer = T*;
  using difference_type = value_type;

  //========================================= SIMPL INTERFACE COMPATIBILITY =================================
  using ContainterType = std::vector<Pointer>;

  //========================================= Constructing DataArray Objects =================================
  DataArray() = default;

  /**
   * @brief Constructor
   * @param numTuples The number of Tuples in the DataArray
   * @param name The name of the DataArray
   * @param initValue The value to use when initializing each element of the array
   */
  DataArray(size_t numTuples, const QString& name, T initValue)
  {
    m_NumComponents = 1;
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
  DataArray(size_t numTuples, const QString& name, comp_dims_type compDims, T initValue)
  : m_NumTuples(numTuples)
  , m_CompDims(std::move(compDims))
  {
    m_NumComponents = std::accumulate(m_CompDims.begin(), m_CompDims.end(), 1, std::multiplies<size_t>());
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
  DataArray(size_t numTuples, const QString& name, comp_dims_type compDims, T initValue, bool allocate)
  : m_NumTuples(numTuples)
  , m_CompDims(std::move(compDims))
  {
    m_NumComponents = std::accumulate(m_CompDims.begin(), m_CompDims.end(), 1, std::multiplies<size_t>());
    m_InitValue = static_cast<T>(0);
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

  ~DataArray()
  {
    clear();
  }

  /**
   * @brief setName
   * @param name
   */
  void setName(const QString& name)
  {
    m_Name = name;
  }
  /**
   * @brief getName
   * @return 
   */
  QString getName() const
  {
    return m_Name;
  }

  //========================================= Static Constructing DataArray Objects =================================
  /**
   * @brief Static constructor
   * @param numElements The number of elements in the internal array.
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of DataArrayTemplate<T>
   */
  static Pointer CreateArray(size_t numTuples, const QString& name, bool allocate = true)
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

  /**
   * @brief Static constructor
   * @param numTuples The number of tuples in the array.
   * @param rank The number of dimensions of the attribute on each Tuple
   * @param dims The actual dimensions of the attribute on each Tuple
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of DataArrayTemplate<T>
   */
  static Pointer CreateArray(size_t numTuples, int rank, const size_t* dims, const QString& name, bool allocate = true)
  {
    if(name.isEmpty())
    {
      return NullPointer();
    }
    comp_dims_type cDims(rank);
    for(int i = 0; i < rank; i++)
    {
      cDims[i] = dims[i];
    }
    auto d = new DataArray<T>(numTuples, name, cDims, allocate);
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

  /**
   * @brief Static constructor
   * @param numTuples The number of tuples in the array.
   * @param compDims The actual dimensions of the attribute on each Tuple
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of DataArrayTemplate<T>
   */
  static Pointer CreateArray(size_t numTuples, const comp_dims_type& compDims, const QString& name, bool allocate = true)
  {
    if(name.isEmpty())
    {
      return NullPointer();
    }
    DataArray<T>* d = new DataArray<T>(numTuples, name, compDims, allocate);
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

  /**
   * @brief Static constructor
   * @param numTuples The number of tuples in the array.
   * @param tDims The actual dimensions of the Tuples
   * @param compDims The number of elements in each axis dimension.
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of DataArrayTemplate<T>
   */
  static Pointer CreateArray(const comp_dims_type& tupleDims, const comp_dims_type& compDims, const QString& name, bool allocate = true)
  {
    if(name.isEmpty())
    {
      return NullPointer();
    }

    size_t numTuples = std::accumulate(tupleDims.begin(), tupleDims.end(), 1, std::multiplies<size_t>());

    auto d = new DataArray<T>(numTuples, name, compDims, allocate);
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

  //========================================= Instance Constructing DataArray Objects =================================
  /**
   * @brief createNewArray Creates a new DataArray object using the same POD type as the existing instance
   * @param numTuples The number of tuples in the array.
   * @param rank The number of dimensions of the attribute on each Tuple
   * @param dims The actual dimensions of the attribute on each Tuple
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return
   */
  Pointer createNewArray(size_t numTuples, int rank, const size_t* compDims, const QString& name, bool allocate = true)
  {
    Pointer p = DataArray<T>::CreateArray(numTuples, rank, compDims, name, allocate);
    return p;
  }

  /**
   * @brief createNewArray
   * @param numTuples The number of tuples in the array.
   * @param compDims The number of elements in each axis dimension.
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of DataArrayTemplate<T>
   */
  Pointer createNewArray(size_t numTuples, const comp_dims_type& compDims, const QString& name, bool allocate = true)
  {
    Pointer p = DataArray<T>::CreateArray(numTuples, compDims, name, allocate);
    return p;
  }

  //========================================= Begin API =================================
  /**
   * @brief GetTypeName Returns a string representation of the type of data that is stored by this class. This
   * can be a primitive like char, float, int or the name of a class.
   * @return
   */
  void getXdmfTypeAndSize(QString& xdmfTypeName, int& precision)
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

  /**
   * @brief GetTypeName Returns a string representation of the type of data that is stored by this class. This
   * can be a primitive like char, float, int or the name of a class.
   * @return
   */

  EbsdLib::NumericTypes::Type getType()
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

  /**
   * @brief Static Method to create a DataArray from a QVector through a deep copy of the data
   * contained in the vector. The number of components will be set to 1.
   * @param vec The vector to copy the data from
   * @param name The name of the array
   * @return Std::Shared_Ptr wrapping an instance of DataArrayTemplate<T>
   */
  static Pointer FromQVector(QVector<T>& vec, const QString& name, bool allocate = true)
  {

    Pointer p = CreateArray(vec.size(), name, allocate);
    if(nullptr != p.get())
    {
      std::memcpy(p->getPointer(0), vec.data(), vec.size() * sizeof(T));
    }
    return p;
  }

  /**
   * @brief Static Method to create a DataArray from a std::vector through a deep copy of the data
   * contained in the vector. The number of components will be set to 1.
   * @param vec The vector to copy the data from
   * @param name The name of the array
   * @return Std::Shared_Ptr wrapping an instance of DataArrayTemplate<T>
   */
  static Pointer FromStdVector(std::vector<T>& vec, const QString& name, bool allocate = true)
  {
    comp_dims_type cDims(1, 1);
    Pointer p = CreateArray(vec.size(), cDims, name, allocate);
    if(nullptr != p.get())
    {
      std::memcpy(p->getPointer(0), &(vec.front()), vec.size() * sizeof(T));
    }
    return p;
  }

  /**
   * @brief FromPointer Creates a DataArray<T> object with a <b>DEEP COPY</b> of the data
   * @param data
   * @param size
   * @param name
   * @return
   */
  static Pointer CopyFromPointer(T* data, size_t size, const QString& name, bool allocate = true)
  {
    Pointer p = CreateArray(size, name, allocate);
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
  static Pointer WrapPointer(T* data, size_t numTuples, const comp_dims_type& compDims, const QString& name, bool ownsData)
  {
    // Allocate on the heap
    auto d = new DataArray(numTuples, name, compDims, false);
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

  /**
   * @brief copyData This method copies the number of tuples specified by the
   * totalSrcTuples value starting from the source tuple offset value in <b>sourceArray</b>
   * into the current array starting at the target destination tuple offset value.
   *
   * For example if the DataArray has 10 tuples, the source DataArray has 10 tuples,
   *  the destTupleOffset = 5, the srcTupleOffset = 5, and the totalSrcTuples = 3,
   *  then tuples 5, 6, and 7 will be copied from the source into tuples 5, 6, and 7
   * of the destination array. In psuedo code it would be the following:
   * @code
   *  destArray[5] = sourceArray[5];
   *  destArray[6] = sourceArray[6];
   *  destArray[7] = sourceArray[7];
   *  .....
   * @endcode
   * @param destTupleOffset
   * @param sourceArray
   * @return
   */
  bool copyFromArray(size_t destTupleOffset, Pointer sourceArray, size_t srcTupleOffset, size_t totalSrcTuples)
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

    if(totalSrcTuples * sourceArray->getNumberOfComponents() + destTupleOffset * getNumberOfComponents() > m_Size)
    {
      return false;
    }

    size_t elementStart = destTupleOffset * getNumberOfComponents();
    size_t totalBytes = (totalSrcTuples * sourceArray->getNumberOfComponents()) * sizeof(T);
    std::memcpy(m_Array + elementStart, source->getPointer(srcTupleOffset * sourceArray->getNumberOfComponents()), totalBytes);
    return true;
  }

  /**
   * @brief copyIntoArray
   * @param dest
   */
  bool copyIntoArray(Pointer dest)
  {
    if(m_IsAllocated && dest->isAllocated() && m_Array && dest->getPointer(0))
    {
      size_t totalBytes = m_Size * sizeof(T);
      std::memcpy(dest->getPointer(0), m_Array, totalBytes);
      return true;
    }
    return false;
  }

  /**
   * @brief isAllocated
   * @return
   */
  bool isAllocated()
  {
    return m_IsAllocated;
  }

  /**
   * @brief Gives this array a human readable name
   * @param name The name of this array
   */
  virtual void setInitValue(T initValue)
  {
    m_InitValue = initValue;
  }

  /**
   * @brief Makes this class responsible for freeing the memory
   */
  void takeOwnership()
  {
    m_OwnsData = true;
  }

  /**
   * @brief This class will NOT free the memory associated with the internal pointer.
   * This can be useful if the user wishes to keep the data around after this
   * class goes out of scope.
   */
  void releaseOwnership()
  {
    m_OwnsData = false;
  }

  /**
   * @brief Allocates the memory needed for this class
   * @return 1 on success, -1 on failure
   */
  virtual int32_t allocate()
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
#if defined(AIM_USE_SSE) && defined(__SSE2__)
    m_Array = static_cast<T*>(_mm_malloc(newSize * sizeof(T), 16));
#else
    m_Array = (T*)malloc(newSize * sizeof(T));
#endif
    if(!m_Array)
    {
      qDebug() << "Unable to allocate " << newSize << " elements of size " << sizeof(T) << " bytes. ";
      return -1;
    }
    m_Size = newSize;
    m_IsAllocated = true;

    return 1;
  }

  /**
   * @brief Sets all the values to zero.
   */
  void initializeWithZeros()
  {
    if(!m_IsAllocated || nullptr == m_Array)
    {
      return;
    }
    size_t typeSize = sizeof(T);
    ::memset(m_Array, 0, m_Size * typeSize);
  }

  /**
   * @brief Sets all the values to value.
   */
  virtual void initializeWithValue(T initValue, size_t offset = 0)
  {
    if(!m_IsAllocated || nullptr == m_Array)
    {
      return;
    }
    std::for_each(begin() + offset, end(), [=](T& n) { n = initValue; });
  }

  /**
   * @brief Removes Tuples from the m_Array. If the size of the vector is Zero nothing is done. If the size of the
   * vector is greater than or Equal to the number of Tuples then the m_Array is Resized to Zero. If there are
   * indices that are larger than the size of the original (before erasing operations) then an error code (-100) is
   * returned from the program.
   * @param idxs The indices to remove
   * @return error code.
   */
  int eraseTuples(comp_dims_type& idxs)
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
    T* newArray = (T*)malloc(newSize * sizeof(T));
    // Splat AB across the array so we know if we are copying the values or not
    ::memset(newArray, 0xAB, newSize * sizeof(T));

    // Keep the current Destination Pointer
    T* currentDest = newArray;
    size_t j = 0;
    int k = 0;
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

    for(int i = 1; i < srcIdx.size(); ++i)
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
    for(int i = 0; i < srcIdx.size(); ++i)
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

  /**
   * @brief
   * @param currentPos
   * @param newPos
   * @return
   */
  int copyTuple(size_t currentPos, size_t newPos)
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

  /**
   * @brief Returns the number of bytes that make up the data type.
   * 1 = char
   * 2 = 16 bit integer
   * 4 = 32 bit integer/Float
   * 8 = 64 bit integer/Double
   */
  size_t getTypeSize()
  {
    return sizeof(T);
  }

  /**
   * @brief Returns the number of elements in the internal array.
   */
  size_t getNumberOfTuples()
  {
    return m_NumTuples;
  }

  /**
   * @brief Returns the total number of elements that make up this array. Equal to NumTuples * NumComponents
   */
  size_t getSize()
  {
    return m_Size;
  }

  /**
   * @brief Returns the dimensions for the data residing at each Tuple. For example if you have a simple Scalar value
   * at each tuple then this will return a single element QVector. If you have a 1x3 array (like EUler Angles) then
   * this will return a 3 Element QVector.
   */
  comp_dims_type getComponentDimensions()
  {
    return m_CompDims;
  }

  /**
   * @brief Returns the number component values at each Tuple location. For example if you have a
   * 3 element component (vector) then this will be 3. If you are storing a small image of size 80x60
   * at each Tuple (like EBSD Kikuchi patterns) then the result would be 4800.
   */
  int getNumberOfComponents()
  {
    return m_NumComponents;
  }

  /**
   * @brief Returns a void pointer pointing to the index of the array. nullptr
   * pointers are entirely possible. No checks are performed to make sure
   * the index is with in the range of the internal data array.
   * @param i The index to have the returned pointer pointing to.
   * @return Void Pointer. Possibly nullptr.
   */
  void* getVoidPointer(size_t i)
  {
    if(i >= m_Size)
    {
      return nullptr;
    }

    return (void*)(&(m_Array[i]));
  }

  /**
   * @brief Returns a list of the contents of DataArray (For Python Binding)
   * @return std::list. Possibly empty
   */
  std::list<T> getArray()
  {
    return std::list<T>(m_Array, m_Array + (m_Size * sizeof(T)) / sizeof(T));
  }

  /**
   * @brief Sets the contents of the array to the list (For Python Binding)
   * @param std::list. New array contents
   */
  void setArray(std::list<T> newArray)
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

  /**
   * @brief Returns the pointer to a specific index into the array. No checks are made
   * as to the correctness of the index being passed in. If you ask for an index off
   * then end of the array they you will likely cause your program to abort.
   * @param i The index to return the pointer to.
   * @return The pointer to the index
   */
  virtual T* getPointer(size_t i)
  {
#ifndef NDEBUG
    if(m_Size > 0)
    {
      Q_ASSERT(i < m_Size);
    }
#endif
    return (T*)(&(m_Array[i]));
  }

  /**
   * @brief Returns the value for a given index
   * @param i The index to return the value at
   * @return The value at index i
   */
  virtual T getValue(size_t i)
  {
#ifndef NDEBUG
    if(m_Size > 0)
    {
      Q_ASSERT(i < m_Size);
    }
#endif
    return m_Array[i];
  }

  /**
   * @brief Sets a specific value in the array
   * @param i The index of the value to set
   * @param value The new value to be set at the specified index
   */
  void setValue(size_t i, T value)
  {
#ifndef NDEBUG
    if(m_Size > 0)
    {
      Q_ASSERT(i < m_Size);
    }
#endif
    m_Array[i] = value;
  }

  //----------------------------------------------------------------------------
  // These can be overridden for more efficiency
  T getComponent(size_t i, int j)
  {
#ifndef NDEBUG
    if(m_Size > 0)
    {
      Q_ASSERT(i * m_NumComponents + j < m_Size);
    }
#endif
    return m_Array[i * m_NumComponents + j];
  }

  /**
   * @brief Sets a specific component of the Tuple located at i
   * @param i The index of the Tuple
   * @param j The Component index into the Tuple
   * @param c The value to set
   */
  void setComponent(size_t i, int j, T c)
  {
#ifndef NDEBUG
    if(m_Size > 0)
    {
      Q_ASSERT(i * m_NumComponents + j < m_Size);
    }
#endif
    m_Array[i * m_NumComponents + j] = c;
  }

  /**
   * @brief setTuple
   * @param tupleIndex
   * @param data
   */
  void setTuple(size_t tupleIndex, T* data)
  {
#ifndef NDEBUG
    if(m_Size > 0)
    {
      Q_ASSERT(tupleIndex * m_NumComponents + (m_NumComponents - 1) < m_Size);
    }
#endif
    std::memcpy(getTuplePointer(tupleIndex), data, m_NumComponents * sizeof(T));
  }

  /**
   * @brief setTuple
   * @param tupleIndex
   * @param data
   */
  void setTuple(size_t tupleIndex, const std::vector<T>& data)
  {
#ifndef NDEBUG
    if(m_Size > 0)
    {
      Q_ASSERT(tupleIndex * m_NumComponents + (m_NumComponents - 1) < m_Size);
    }
#endif
    std::memcpy(getTuplePointer(tupleIndex), data.data(), m_NumComponents * sizeof(T));
  }

  /**
   * @brief Splats the same value c across all values in the Tuple
   * @param i The index of the Tuple
   * @param c The value to splat across all components in the tuple
   */
  void initializeTuple(size_t i, void* p)
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

  /**
   * @brief getTuplePointer Returns the pointer to a specific tuple
   * @param tupleIndex The index of tuple
   */
  T* getTuplePointer(size_t tupleIndex)
  {
#ifndef NDEBUG
    if(m_Size > 0)
    {
      Q_ASSERT(tupleIndex * m_NumComponents < m_Size);
    }
#endif
    return m_Array + (tupleIndex * m_NumComponents);
  }

  /**
   * @brief resize
   * @param numTuples
   * @return
   */
  void resizeTuples(size_t numTuples)
  {
    T* ptr = resizeAndExtend(numTuples * m_NumComponents);
    if(nullptr != ptr)
    {
      m_NumTuples = numTuples;
    }
  }

  /**
   * @brief printTuple
   * @param out
   * @param i
   * @param delimiter
   */
  void printTuple(QTextStream& out, size_t i, char delimiter = ',')
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

  /**
   * @brief printComponent
   * @param out
   * @param i
   * @param j
   */
  void printComponent(QTextStream& out, size_t i, int j)
  {
    out << m_Array[i * m_NumComponents + j];
  }

  /**
   * @brief Returns the HDF Type for a given primitive value.
   * @param value A value to use. Can be anything. Just used to get the type info
   * from
   * @return 
   */
  QString getFullNameOfClass()
  {
    QString theType = getTypeAsString();
    theType = "DataArray<" + theType + ">";
    return theType;
  }

  /**
   * @brief getTypeAsString
   * @return
   */
  QString getTypeAsString()
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
      return "char";
    }
#else
    if(typeid(value) == typeid(char))
    {
      return "char";
    }
#endif
    if(typeid(value) == typeid(signed char))
    {
      return "signed char";
    }
    if(typeid(value) == typeid(unsigned char))
    {
      return "unsigned char";
    }

    if(typeid(value) == typeid(int16_t))
    {
      return "int16_t";
    }
    if(typeid(value) == typeid(short))
    {
      return "short";
    }
    if(typeid(value) == typeid(signed short))
    {
      return "signed short";
    }
    if(typeid(value) == typeid(uint16_t))
    {
      return "uint16_t";
    }
    if(typeid(value) == typeid(unsigned short))
    {
      return "unsigned short";
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
      return "int";
    }
    if(typeid(value) == typeid(signed int))
    {
      return "signed int";
    }
    if(typeid(value) == typeid(unsigned int))
    {
      return "unsigned int";
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
#endif

#if(CMP_SIZEOF_LONG_LONG == 8)
    if(typeid(value) == typeid(long long int))
    {
      return "long long int";
    }
    if(typeid(value) == typeid(signed long long int))
    {
      return "signed long long int";
    }
    if(typeid(value) == typeid(unsigned long long int))
    {
      return "unsigned long long int";
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

  /**
   * @brief deepCopy
   * @param forceNoAllocate
   * @return
   */
  Pointer deepCopy(bool forceNoAllocate = false)
  {
    Pointer daCopy = createNewArray(getNumberOfTuples(), getComponentDimensions(), getName(), m_IsAllocated);
    if(m_IsAllocated && !forceNoAllocate)
    {
      T* src = getPointer(0);
      void* dest = daCopy->getVoidPointer(0);
      size_t totalBytes = (getNumberOfTuples() * getNumberOfComponents() * sizeof(T));
      std::memcpy(dest, src, totalBytes);
    }
    return daCopy;
  }

#ifdef EbsdLib_ENABLE_HDF5_OFF
  /**
   *
   * @param parentId
   * @return
   */
  int writeH5Data(hid_t parentId, comp_dims_type tDims)
  {
    if(m_Array == nullptr)
    {
      return -85648;
    }
    return H5DataArrayWriter::writeDataArray<Self>(parentId, this, tDims);
  }
#endif

  /**
   * @brief writeXdmfAttribute
   * @param out
   * @param volDims
   * @return
   */
  int writeXdmfAttribute(QTextStream& out, int64_t* volDims, const QString& hdfFileName, const QString& groupPath, const QString& label)
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

  /**
   * @brief getInfoString
   * @return Returns a formatted string that contains general infomation about
   * the instance of the object.
   */
  QString getInfoString(EbsdLib::InfoStringFormat format)
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
      for(int i = 0; i < m_CompDims.size(); i++)
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
    else
    {
    }
    return info;
  }

#ifdef EbsdLib_ENABLE_HDF5_OFF
  /**
   * @brief
   * @param parentId
   * @return
   */
  int readH5Data(hid_t parentId)
  {
    int err = 0;

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
    return err;
  }
#endif

  /**
   * @brief
   */
  virtual void byteSwapElements()
  {
    char* ptr = (char*)(m_Array);
    char t[8];
    size_t size = getTypeSize();
    for(uint64_t var = 0; var < m_Size; ++var)
    {
      if(sizeof(T) == 2)
      {
        mxa_bswap(0, 1, t);
      }
      else if(sizeof(T) == 4)
      {
        mxa_bswap(0, 3, t);
        mxa_bswap(1, 2, t);
      }
      else if(sizeof(T) == 8)
      {
        mxa_bswap(0, 7, t);
        mxa_bswap(1, 6, t);
        mxa_bswap(2, 5, t);
        mxa_bswap(3, 4, t);
      }
      ptr += size; // increment the pointer
    }
  }

  //========================================= STL INTERFACE COMPATIBILITY =================================

  class tuple_iterator
  {
  public:
    using self_type = tuple_iterator;
    using value_type = T;
    using reference = T&;
    using pointer = T*;
    using difference_type = value_type;
    using iterator_category = std::forward_iterator_tag;

    tuple_iterator(pointer ptr, size_type numComps)
    : ptr_(ptr)
    , num_comps_(numComps)
    {
    }
    self_type operator++()
    {
      ptr_ = ptr_ + num_comps_;
      return *this;
    } // PREFIX
    self_type operator++(int junk)
    {
      self_type i = *this;
      ptr_ = ptr_ + num_comps_;
      return i;
    } // POSTFIX
    reference operator*()
    {
      return *ptr_;
    }
    pointer operator->()
    {
      return ptr_;
    }
    bool operator==(const self_type& rhs)
    {
      return ptr_ == rhs.ptr_;
    }
    bool operator!=(const self_type& rhs)
    {
      return ptr_ != rhs.ptr_;
    }
    reference comp_value(size_type comp)
    {
      return *(ptr_ + comp);
    }

  private:
    pointer ptr_;
    size_t num_comps_;
  };

  class const_tuple_iterator
  {
  public:
    using self_type = const_tuple_iterator;
    using value_type = T;
    using reference = T&;
    using pointer = T*;
    using difference_type = value_type;
    using iterator_category = std::forward_iterator_tag;

    const_tuple_iterator(pointer ptr, size_type numComps)
    : ptr_(ptr)
    , num_comps_(numComps)
    {
    }
    self_type operator++()
    {
      ptr_ = ptr_ + num_comps_;
      return *this;
    } // PREFIX
    self_type operator++(int junk)
    {
      self_type i = *this;
      ptr_ = ptr_ + num_comps_;
      return i;
    } // POSTFIX
    const value_type& operator*()
    {
      return *ptr_;
    }
    const pointer operator->()
    {
      return ptr_;
    }
    bool operator==(const self_type& rhs)
    {
      return ptr_ == rhs.ptr_;
    }
    bool operator!=(const self_type& rhs)
    {
      return ptr_ != rhs.ptr_;
    }
    const value_type& comp_value(size_type comp)
    {
      return *(ptr_ + comp);
    }

  private:
    pointer ptr_;
    size_t num_comps_;
  };

  class iterator
  {
  public:
    using self_type = iterator;
    using value_type = T;
    using reference = T&;
    using pointer = T*;
    using difference_type = value_type;
    using iterator_category = std::forward_iterator_tag;

    iterator(pointer ptr)
    : ptr_(ptr)
    {
    }
    iterator(pointer ptr, size_type ununsed)
    : ptr_(ptr)
    {
    }

    self_type operator++()
    {
      ptr_++;
      return *this;
    } // PREFIX
    self_type operator++(int junk)
    {
      self_type i = *this;
      ptr_++;
      return i;
    } // POSTFIX
    self_type operator+(int amt)
    {
      ptr_ += amt;
      return *this;
    }
    reference operator*()
    {
      return *ptr_;
    }
    pointer operator->()
    {
      return ptr_;
    }
    bool operator==(const self_type& rhs)
    {
      return ptr_ == rhs.ptr_;
    }
    bool operator!=(const self_type& rhs)
    {
      return ptr_ != rhs.ptr_;
    }

  private:
    pointer ptr_;
  };

  class const_iterator
  {
  public:
    using self_type = const_iterator;
    using value_type = T;
    using reference = T&;
    using pointer = T*;
    using difference_type = value_type;
    using iterator_category = std::forward_iterator_tag;
    const_iterator(pointer ptr)
    : ptr_(ptr)
    {
    }
    const_iterator(pointer ptr, size_type unused)
    : ptr_(ptr)
    {
    }

    self_type operator++()
    {
      ptr_++;
      return *this;
    } // PREFIX
    self_type operator++(int amt)
    {
      self_type i = *this;
      ptr_ += amt;
      return i;
    } // POSTFIX
    self_type operator+(int amt)
    {
      ptr_ += amt;
      return *this;
    }
    const value_type& operator*()
    {
      return *ptr_;
    }
    const pointer operator->()
    {
      return ptr_;
    }
    bool operator==(const self_type& rhs)
    {
      return ptr_ == rhs.ptr_;
    }
    bool operator!=(const self_type& rhs)
    {
      return ptr_ != rhs.ptr_;
    }

  private:
    pointer ptr_;
  };

  // ######### Iterators #########

  template <typename IteratorType>
  IteratorType begin()
  {
    return IteratorType(m_Array, m_NumComponents);
  }
  iterator begin()
  {
    return iterator(m_Array);
  }

  template <typename IteratorType>
  IteratorType end()
  {
    return IteratorType(m_Array + m_Size, m_NumComponents);
  }
  iterator end()
  {
    return iterator(m_Array + m_Size);
  }

  const_iterator begin() const
  {
    return const_iterator(m_Array);
  }

  const_iterator end() const
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

  size_type size() const
  {
    return m_Size;
  }

  size_type max_size() const
  {
    return m_Size;
  }
  //  void resize(size_type n)
  //  {
  //    resizeAndExtend(n);
  //  }
  // void resize (size_type n, const value_type& val);
  size_type capacity() const noexcept
  {
    return m_Size;
  }
  bool empty() const noexcept
  {
    return (m_Size == 0);
  }
  // reserve()
  // shrink_to_fit()

  // ######### Element Access #########

  inline reference operator[](size_type index)
  {
    // assert(index < m_Size);
    return m_Array[index];
  }

  inline const T& operator[](size_type index) const
  {
    // assert(index < m_Size);
    return m_Array[index];
  }

  inline reference at(size_type index)
  {
    assert(index < m_Size);
    return m_Array[index];
  }

  inline const T& at(size_type index) const
  {
    assert(index < m_Size);
    return m_Array[index];
  }

  inline reference front()
  {
    return m_Array[0];
  }
  inline const T& front() const
  {
    return m_Array[0];
  }

  inline reference back()
  {
    return m_Array[m_MaxId];
  }
  inline const T& back() const
  {
    return m_Array[m_MaxId];
  }

  inline T* data() noexcept
  {
    return m_Array;
  }
  inline const T* data() const noexcept
  {
    return m_Array;
  }

  // ######### Modifiers #########

  /**
   * @brief In the range version (1), the new contents are elements constructed from each of the elements in the range
   * between first and last, in the same order.
   */
  template <class InputIterator>
  void assign(InputIterator first, InputIterator last) // range (1)
  {
    size_type size = last - first;
    resizeAndExtend(size);
    size_type idx = 0;
    while(first != last)
    {
      m_Array[idx] = *first;
      first++;
    }
  }

  /**
   * @brief In the fill version (2), the new contents are n elements, each initialized to a copy of val.
   * @param n
   * @param val
   */
  void assign(size_type n, const value_type& val) // fill (2)
  {
    resizeAndExtend(n);
    std::for_each(begin(), end(), [=](T& n) { n = val; });
  }

  /**
   * @brief In the initializer list version (3), the new contents are copies of the values passed as initializer list, in the same order.
   * @param il
   */
  void assign(std::initializer_list<value_type> il) //  initializer list (3)
  {
    assign(il.begin(), il.end());
  }

  /**
   * @brief push_back
   * @param val
   */
  void push_back(const value_type& val)
  {
    resizeAndExtend(m_Size + 1);
    m_Array[m_MaxId] = val;
  }
  /**
   * @brief push_back
   * @param val
   */
  void push_back(value_type&& val)
  {
    resizeAndExtend(m_Size + 1);
    m_Array[m_MaxId] = val;
  }

  /**
   * @brief pop_back
   */
  void pop_back()
  {
    resizeAndExtend(m_Size - 1);
  }
  // insert
  // iterator erase (const_iterator position)
  // iterator erase (const_iterator first, const_iterator last);
  // swap

  /**
   * @brief Removes all elements from the array (which are destroyed), leaving the container with a size of 0.
   */
  void clear()
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

  /**
   * @brief equal
   * @param range1
   * @param range2
   * @return
   */
  template <typename Range1, typename Range2>
  bool equal(Range1 const& range1, Range2 const& range2)
  {
    if(range1.size() != range2.size())
    {
      return false;
    }

    return std::equal(begin(range1), end(range1), begin(range2));
  }

  // =================================== END STL COMPATIBLE INTERFACe ===================================================

protected:
  /**
   * @brief deallocates the memory block
   */
  void deallocate()
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

#if defined(AIM_USE_SSE) && defined(__SSE2__)
    _mm_free(m_buffer);
#else
    free(m_Array);
#endif
    m_Array = nullptr;
    m_IsAllocated = false;
  }

  /**
   * @brief Resizes the internal array
   * @param size The new size of the internal array
   * @return 1 on success, 0 on failure
   */
  int32_t resizeTotalElements(size_t size)
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

  /**
   * @brief resizes the internal array to be 'size' elements in length
   * @param size
   * @return Pointer to the internal array
   */
  virtual T* resizeAndExtend(size_t size)
  {
    T* newArray;
    size_t newSize;
    size_t oldSize;

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
    // OS X's realloc does not free memory if the new block is smaller.  This
    // is a very serious problem and causes huge amount of memory to be
    // wasted. Do not use realloc on the Mac.
    bool dontUseRealloc = false;
#if defined __APPLE__
    dontUseRealloc = true;
#endif

    // Allocate a new array if we DO NOT own the current array
    if((nullptr != m_Array) && (false == m_OwnsData))
    {
      // The old array is owned by the user so we cannot try to
      // reallocate it.  Just allocate new memory that we will own.
      newArray = (T*)malloc(newSize * sizeof(T));
      if(!newArray)
      {
        qDebug() << "Unable to allocate " << newSize << " elements of size " << sizeof(T) << " bytes. ";
        return nullptr;
      }

      // Copy the data from the old array.
      std::memcpy(newArray, m_Array, (newSize < m_Size ? newSize : m_Size) * sizeof(T));
    }
    else if(!dontUseRealloc)
    {
      // Try to reallocate with minimal memory usage and possibly avoid copying.
      newArray = (T*)realloc(m_Array, newSize * sizeof(T));
      if(!newArray)
      {
        qDebug() << "Unable to allocate " << newSize << " elements of size " << sizeof(T) << " bytes. ";
        return nullptr;
      }
    }
    else
    {
      newArray = (T*)malloc(newSize * sizeof(T));
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

private:
  T* m_Array = nullptr;
  size_t m_Size = 0;
  size_t m_MaxId = 0;
  size_t m_NumTuples = 0;
  size_t m_NumComponents = 1;
  T m_InitValue = static_cast<T>(0);
  comp_dims_type m_CompDims = {1};
  bool m_IsAllocated = false;
  bool m_OwnsData = true;
  QString m_Name = {};
};

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

using BoolArrayType = DataArray<bool>;

using UCharArrayType = DataArray<unsigned char>;

using Int8ArrayType = DataArray<int8_t>;
using UInt8ArrayType = DataArray<uint8_t>;

using Int16ArrayType = DataArray<int16_t>;
using UInt16ArrayType = DataArray<uint16_t>;

using Int32ArrayType = DataArray<int32_t>;
using UInt32ArrayType = DataArray<uint32_t>;

using Int64ArrayType = DataArray<int64_t>;
using UInt64ArrayType = DataArray<uint64_t>;

using FloatArrayType = DataArray<float>;
using DoubleArrayType = DataArray<double>;

using SizeTArrayType = DataArray<size_t>;
