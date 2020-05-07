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

// STL Includes
#include <cassert>
#include <vector>
#include <iterator>
#include <memory>

#include <QtCore/QString>
#include <QtCore/QTextStream>

#include <hdf5.h>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Core/EbsdLibConstants.h"

/**
 * @class EbsdDataArray
 * @brief Template class for wrapping raw arrays of data and is the basis for storing data within the SIMPL data structure.
 */
template <typename T>
class EbsdDataArray
{

public:
  using Self = EbsdDataArray<T>;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<const Self>;

  static Pointer NullPointer();

  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  QString getNameOfClass() const;
  /**
   * @brief Returns the name of the class for AbstractMessage
   */
  static QString ClassName();

  /**
   * @brief Returns the version of this class.
   * @return
   */
  int32_t getClassVersion() const;

  EbsdDataArray(const EbsdDataArray&) = default;           // Copy Constructor default Implemented
  EbsdDataArray(EbsdDataArray&&) = delete;                 // Move Constructor Not Implemented
  EbsdDataArray& operator=(const EbsdDataArray&) = delete; // Copy Assignment Not Implemented
  EbsdDataArray& operator=(EbsdDataArray&&) = delete;      // Move Assignment Not Implemented

  //========================================= STL INTERFACE COMPATIBILITY =================================
  using comp_dims_type = std::vector<size_t>;
  using size_type = size_t;
  using value_type = T;
  using reference = T&;
  using pointer = T*;
  using difference_type = ptrdiff_t;

  //========================================= SIMPL INTERFACE COMPATIBILITY =================================
  using ContainterType = std::vector<Pointer>;

  //========================================= Constructing EbsdDataArray Objects =================================
  EbsdDataArray();

  /**
   * @brief Constructor
   * @param numTuples The number of Tuples in the EbsdDataArray
   * @param name The name of the EbsdDataArray
   * @param initValue The value to use when initializing each element of the array
   */
  EbsdDataArray(size_t numTuples, const QString& name, T initValue);
  /**
   * @brief EbsdDataArray
   * @param numTuples The number of Tuples in the EbsdDataArray
   * @param name The name of the EbsdDataArray
   * @param compDims The number of elements in each axis dimension.
   * @param initValue The value to use when initializing each element of the array
   *
   * For example if you have a 2D image dimensions of 80(w) x 60(h) then the "cdims" would be [80][60]
   */
  EbsdDataArray(size_t numTuples, const QString& name, const comp_dims_type& compDims, T initValue);

  /**
   * @brief Protected Constructor
   * @param numTuples The number of Tuples in the EbsdDataArray
   * @param name The name of the EbsdDataArray
   * @param compDims The number of elements in each axis dimension.
   * @param initValue The value to use when initializing each element of the array
   * @param allocate Will all the memory be allocated at time of construction
   */
  EbsdDataArray(size_t numTuples, const QString& name, const comp_dims_type& compDims, T initValue, bool allocate);

  ~EbsdDataArray();

  //========================================= Static Constructing EbsdDataArray Objects =================================
  /**
   * @brief Static constructor
   * @param numElements The number of elements in the internal array.
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of EbsdDataArrayTemplate<T>
   */
  static Pointer CreateArray(size_t numTuples, const QString& name, bool allocate);

  /**
   * @brief Static constructor
   * @param numTuples The number of tuples in the array.
   * @param rank The number of dimensions of the attribute on each Tuple
   * @param dims The actual dimensions of the attribute on each Tuple
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of EbsdDataArrayTemplate<T>
   */
  static Pointer CreateArray(size_t numTuples, int32_t rank, const size_t* dims, const QString& name, bool allocate);

  /**
   * @brief Static constructor
   * @param numTuples The number of tuples in the array.
   * @param compDims The actual dimensions of the attribute on each Tuple
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of EbsdDataArrayTemplate<T>
   */
  static Pointer CreateArray(size_t numTuples, const comp_dims_type& compDims, const QString& name, bool allocate);

  /**
   * @brief Static constructor
   * @param numTuples The number of tuples in the array.
   * @param tDims The actual dimensions of the Tuples
   * @param compDims The number of elements in each axis dimension.
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of EbsdDataArrayTemplate<T>
   */
  static Pointer CreateArray(const comp_dims_type& tupleDims, const comp_dims_type& compDims, const QString& name, bool allocate);

  //========================================= Instance Constructing EbsdDataArray Objects =================================
  /**
   * @brief createNewArray Creates a new EbsdDataArray object using the same POD type as the existing instance
   * @param numTuples The number of tuples in the array.
   * @param rank The number of dimensions of the attribute on each Tuple
   * @param dims The actual dimensions of the attribute on each Tuple
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return
   */
  Pointer createNewArray(size_t numTuples, int32_t rank, const size_t* compDims, const QString& name, bool allocate) const;

  /**
   * @brief createNewArray
   * @param numTuples The number of tuples in the array.
   * @param compDims The number of elements in each axis dimension.
   * @param name The name of the array
   * @param allocate Will all the memory be allocated at time of construction
   * @return Std::Shared_Ptr wrapping an instance of EbsdDataArrayTemplate<T>
   */
  Pointer createNewArray(size_t numTuples, const comp_dims_type& compDims, const QString& name, bool allocate) const;

  /**
   * @brief Static Method to create a EbsdDataArray from a QVector through a deep copy of the data
   * contained in the vector. The number of components will be set to 1.
   * @param vec The vector to copy the data from
   * @param name The name of the array
   * @return Std::Shared_Ptr wrapping an instance of EbsdDataArrayTemplate<T>
   */
  static Pointer FromQVector(const QVector<T>& vec, const QString& name);

  /**
   * @brief Static Method to create a EbsdDataArray from a std::vector through a deep copy of the data
   * contained in the vector. The number of components will be set to 1.
   * @param vec The vector to copy the data from
   * @param name The name of the array
   * @return Std::Shared_Ptr wrapping an instance of EbsdDataArrayTemplate<T>
   */
  static Pointer FromStdVector(const std::vector<T>& vec, const QString& name);

  /**
   * @brief FromPointer Creates a EbsdDataArray<T> object with a <b>DEEP COPY</b> of the data
   * @param data
   * @param size
   * @param name
   * @return
   */
  static Pointer CopyFromPointer(const T* data, size_t size, const QString& name);

  /**
   * @brief WrapPointer Creates a EbsdDataArray<T> object that references the pointer. The original caller can
   * set if the memory should be "free()'ed" when the object goes away. The original memory MUST have been
   * "alloc()'ed" and <b>NOT</b> new 'ed.
   * @param data
   * @param numTuples
   * @param cDims
   * @param name
   * @param ownsData
   * @return
   */
  static Pointer WrapPointer(T* data, size_t numTuples, const comp_dims_type& compDims, const QString& name, bool ownsData);

  /**
   * @brief Use this method to move the pointer ownership from this class to another similar class, such as SIMPLib::DataArray<T>
   */
  template <typename DataArrayType>
  std::shared_ptr<DataArrayType> moveToDataArrayType()
  {
    std::shared_ptr<DataArrayType> output = DataArrayType::WrapPointer(data(), getNumberOfTuples(), getComponentDimensions(), getName(), true);
    releaseOwnership();
    return output;
  }

  //========================================= Begin API =================================

  /**
   * @brief setName
   * @param name
   */
  void setName(const QString& name);

  /**
   * @brief getName
   * @return
   */
  QString getName() const;

  /**
   * @brief deepCopy
   * @param forceNoAllocate
   * @return
   */
  Pointer deepCopy(bool forceNoAllocate = false) const;

  /**
   * @brief GetTypeName Returns a string representation of the type of data that is stored by this class. This
   * can be a primitive like char, float, int or the name of a class.
   * @return
   */

  EbsdLib::NumericTypes::Type getType() const;

  /**
   * @brief GetTypeName Returns a string representation of the type of data that is stored by this class. This
   * can be a primitive like char, float, int or the name of a class.
   * @return
   */
  void getXdmfTypeAndSize(QString& xdmfTypeName, int32_t& precision) const;

  /**
   * @brief copyData This method copies the number of tuples specified by the
   * totalSrcTuples value starting from the source tuple offset value in <b>sourceArray</b>
   * into the current array starting at the target destination tuple offset value.
   *
   * For example if the EbsdDataArray has 10 tuples, the source EbsdDataArray has 10 tuples,
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
  bool copyFromArray(size_t destTupleOffset, ConstPointer sourceArray, size_t srcTupleOffset, size_t totalSrcTuples);

  /**
   * @brief copyIntoArray
   * @param dest
   */
  bool copyIntoArray(Pointer dest) const;

  /**
   * @brief isAllocated
   * @return
   */
  bool isAllocated() const;
  /**
   * @brief Gives this array a human readable name
   * @param name The name of this array
   */
  void setInitValue(T initValue);

  /**
   * @brief Returns the initial value for the array.
   * @return
   */
  T getInitValue() const
  {
    return m_InitValue;
  }

  /**
   * @brief Makes this class responsible for freeing the memory
   */
  void takeOwnership();

  /**
   * @brief This class will NOT free the memory associated with the internal pointer.
   * This can be useful if the user wishes to keep the data around after this
   * class goes out of scope.
   */
  void releaseOwnership();

  /**
   * @brief Allocates the memory needed for this class
   * @return 1 on success, -1 on failure
   */
  int32_t allocate();

  /**
   * @brief Sets all the values to zero.
   */
  void initializeWithZeros();

  /**
   * @brief Sets all the values to value.
   */
  void initializeWithValue(T initValue, size_t offset = 0);

  /**
   * @brief Removes Tuples from the m_Array. If the size of the vector is Zero nothing is done. If the size of the
   * vector is greater than or Equal to the number of Tuples then the m_Array is Resized to Zero. If there are
   * indices that are larger than the size of the original (before erasing operations) then an error code (-100) is
   * returned from the program.
   * @param idxs The indices to remove
   * @return error code.
   */
  int32_t eraseTuples(const comp_dims_type& idxs);

  /**
   * @brief
   * @param currentPos
   * @param newPos
   * @return
   */
  int32_t copyTuple(size_t currentPos, size_t newPos);

  /**
   * @brief Returns the number of bytes that make up the data type.
   * 1 = char
   * 2 = 16 bit integer
   * 4 = 32 bit integer/Float
   * 8 = 64 bit integer/Double
   */
  size_t getTypeSize() const;

  /**
   * @brief Returns the number of elements in the internal array.
   */
  size_t getNumberOfTuples() const;

  /**
   * @brief Returns the total number of elements that make up this array. Equal to NumTuples * NumComponents
   */
  size_t getSize() const;

  /**
   * @brief Returns the dimensions for the data residing at each Tuple. For example if you have a simple Scalar value
   * at each tuple then this will return a single element QVector. If you have a 1x3 array (like EUler Angles) then
   * this will return a 3 Element QVector.
   */
  comp_dims_type getComponentDimensions() const;

  /**
   * @brief Returns the number component values at each Tuple location. For example if you have a
   * 3 element component (vector) then this will be 3. If you are storing a small image of size 80x60
   * at each Tuple (like EBSD Kikuchi patterns) then the result would be 4800.
   */
  int32_t getNumberOfComponents() const;

  /**
   * @brief Returns a void pointer pointing to the index of the array. nullptr
   * pointers are entirely possible. No checks are performed to make sure
   * the index is with in the range of the internal data array.
   * @param i The index to have the returned pointer pointing to.
   * @return Void Pointer. Possibly nullptr.
   */
  void* getVoidPointer(size_t i);

  /**
   * @brief Returns the pointer to a specific index into the array. No checks are made
   * as to the correctness of the index being passed in. If you ask for an index off
   * then end of the array they you will likely cause your program to abort.
   * @param i The index to return the pointer to.
   * @return The pointer to the index
   */
  T* getPointer(size_t i) const;

  /**
   * @brief Returns the value for a given index
   * @param i The index to return the value at
   * @return The value at index i
   */
  T getValue(size_t i) const;

  /**
   * @brief Sets a specific value in the array
   * @param i The index of the value to set
   * @param value The new value to be set at the specified index
   */
  void setValue(size_t i, T value);

  //----------------------------------------------------------------------------
  // These can be overridden for more efficiency
  T getComponent(size_t i, int32_t j) const;

  /**
   * @brief Sets a specific component of the Tuple located at i
   * @param i The index of the Tuple
   * @param j The Component index into the Tuple
   * @param c The value to set
   */
  void setComponent(size_t i, int32_t j, T c);

  /**
   * @brief setTuple
   * @param tupleIndex
   * @param data
   */
  void setTuple(size_t tupleIndex, const T* data);

  /**
   * @brief setTuple
   * @param tupleIndex
   * @param data
   */
  void setTuple(size_t tupleIndex, const std::vector<T>& data);

  /**
   * @brief Splats the same value c across all values in the Tuple
   * @param i The index of the Tuple
   * @param c The value to splat across all components in the tuple
   */
  void initializeTuple(size_t i, const void* p);

  /**
   * @brief getTuplePointer Returns the pointer to a specific tuple
   * @param tupleIndex The index of tuple
   */
  T* getTuplePointer(size_t tupleIndex) const;

  /**
   * @brief resize
   * @param numTuples
   * @return
   */
  void resizeTuples(size_t numTuples);

  /**
   * @brief printTuple
   * @param out
   * @param i
   * @param delimiter
   */
  void printTuple(QTextStream& out, size_t i, char delimiter = ',') const;

  /**
   * @brief printComponent
   * @param out
   * @param i
   * @param j
   */
  void printComponent(QTextStream& out, size_t i, int32_t j) const;

  /**
   * @brief Returns the HDF Type for a given primitive value.
   * @param value A value to use. Can be anything. Just used to get the type info
   * from
   * @return The HDF5 native type for the value
   */
  QString getFullNameOfClass() const;

  /**
   * @brief getTypeAsString
   * @return
   */
  QString getTypeAsString() const;

#ifdef DATA_ARRAY_ENABLE_HDF5_IO
  /**
   *
   * @param parentId
   * @return
   */
  int32_t writeH5Data(hid_t parentId, comp_dims_type tDims) const;
#endif
  /**
   * @brief writeXdmfAttribute
   * @param out
   * @param volDims
   * @return
   */
  int writeXdmfAttribute(QTextStream& out, const int64_t* volDims, const QString& hdfFileName, const QString& groupPath, const QString& label) const;

#ifdef DATA_ARRAY_ENABLE_ToolTipGenerator
  /**
   * @brief Returns a ToolTipGenerator for creating HTML tooltip tables
   * with values populated to match the current EbsdDataArray.
   * @return
   */
  ToolTipGenerator getToolTipGenerator() const
  {
    ToolTipGenerator toolTipGen;
    QLocale usa(QLocale::English, QLocale::UnitedStates);

    toolTipGen.addTitle("Attribute Array Info");
    toolTipGen.addValue("Name", getName());
    toolTipGen.addValue("Type", getTypeAsString());
    toolTipGen.addValue("Number of Tuples", usa.toString(static_cast<qlonglong>(getNumberOfTuples())));

    QString compDimStr = "(";
    for(int i = 0; i < m_CompDims.size(); i++)
    {
      compDimStr = compDimStr + QString::number(m_CompDims[i]);
      if(i < m_CompDims.size() - 1)
      {
        compDimStr = compDimStr + QString(", ");
      }
    }
    compDimStr += ")";
    toolTipGen.addValue("Component Dimensions", compDimStr);
    toolTipGen.addValue("Total Elements", usa.toString(static_cast<qlonglong>(m_Size)));
    toolTipGen.addValue("Total Memory Required", usa.toString(static_cast<qlonglong>(m_Size * sizeof(T))));

    return toolTipGen;
  }
#endif

  /**
   * @brief getInfoString
   * @return Returns a formatted string that contains general infomation about
   * the instance of the object.
   */
  QString getInfoString(EbsdLib::InfoStringFormat format) const;

  /**
   * @brief
   * @param parentId
   * @return
   */
  int readH5Data(hid_t parentId);

  /**
   * @brief
   */
  void byteSwapElements();

  //========================================= STL INTERFACE COMPATIBILITY =================================

  class tuple_iterator
  {
  public:
    using self_type = tuple_iterator;
    using value_type = T;
    using reference = T&;
    using pointer = T*;
    using difference_type = ptrdiff_t;
    using iterator_category = std::random_access_iterator_tag;

    tuple_iterator() = delete;

    self_type& operator++()
    {
      ptr_ += num_comps_;
      return *this;
    } // PREFIX
    self_type operator++(int32_t unused)
    {
      // Replace with [[maybe_unused]] in C++ 17
      (void)unused;
      self_type i = *this;
      ptr_ += num_comps_;
      return i;
    } // POSTFIX
    self_type& operator--()
    {
      ptr_ -= num_comps_;
      return *this;
    } // PREFIX
    self_type operator--(int32_t unused)
    {
      // Replace with [[maybe_unused]] in C++ 17
      (void)unused;
      self_type i = *this;
      ptr_ -= num_comps_;
      return i;
    } // POSTFIX
    self_type& operator+=(difference_type amt)
    {
      ptr_ += amt * num_comps_;
      return *this;
    }
    self_type operator+(difference_type amt) const
    {
      return self_type(ptr_ + amt * num_comps_, num_comps_);
    }
    self_type& operator-=(difference_type amt)
    {
      ptr_ -= amt * num_comps_;
      return *this;
    }
    self_type operator-(difference_type amt) const
    {
      return self_type(ptr_ - amt * num_comps_, num_comps_);
    }
    difference_type operator-(const self_type& rhs) const
    {
      return (ptr_ - rhs.ptr_) / static_cast<difference_type>(num_comps_);
    }
    friend self_type operator+(difference_type lhs, const self_type& rhs)
    {
      return self_type(lhs * rhs.num_comps_ + rhs.ptr_, rhs.num_comps_);
    }
    reference operator[](difference_type amt) const
    {
      return ptr_[amt * num_comps_];
    }
    reference operator*() const
    {
      return *ptr_;
    }
    pointer operator->() const
    {
      return ptr_;
    }
    bool operator==(const self_type& rhs) const
    {
      return ptr_ == rhs.ptr_;
    }
    bool operator!=(const self_type& rhs) const
    {
      return ptr_ != rhs.ptr_;
    }
    bool operator>(const self_type& rhs) const
    {
      return ptr_ > rhs.ptr_;
    }
    bool operator<(const self_type& rhs) const
    {
      return ptr_ < rhs.ptr_;
    }
    bool operator>=(const self_type& rhs) const
    {
      return ptr_ >= rhs.ptr_;
    }
    bool operator<=(const self_type& rhs) const
    {
      return ptr_ <= rhs.ptr_;
    }

    reference comp_value(size_type comp)
    {
      return *(ptr_ + comp);
    }

  protected:
    tuple_iterator(pointer ptr, size_type numComps)
    : ptr_(ptr)
    , num_comps_(numComps)
    {
    }

    friend class EbsdDataArray;

  private:
    pointer ptr_;
    size_t num_comps_;
  };

  class const_tuple_iterator
  {
  public:
    using self_type = const_tuple_iterator;
    using value_type = T;
    using reference = const T&;
    using pointer = const T*;
    using difference_type = ptrdiff_t;
    using iterator_category = std::random_access_iterator_tag;

    const_tuple_iterator() = delete;

    self_type& operator++()
    {
      ptr_ += num_comps_;
      return *this;
    } // PREFIX
    self_type operator++(int32_t unused)
    {
      // Replace with [[maybe_unused]] in C++ 17
      (void)unused;
      self_type i = *this;
      ptr_ += num_comps_;
      return i;
    } // POSTFIX
    self_type& operator--()
    {
      ptr_ -= num_comps_;
      return *this;
    } // PREFIX
    self_type operator--(int32_t unused)
    {
      // Replace with [[maybe_unused]] in C++ 17
      (void)unused;
      self_type i = *this;
      ptr_ -= num_comps_;
      return i;
    } // POSTFIX
    self_type& operator+=(difference_type amt)
    {
      ptr_ += amt * num_comps_;
      return *this;
    }
    self_type operator+(difference_type amt) const
    {
      return self_type(ptr_ + amt * num_comps_, num_comps_);
    }
    self_type& operator-=(difference_type amt)
    {
      ptr_ -= amt * num_comps_;
      return *this;
    }
    self_type operator-(difference_type amt) const
    {
      return self_type(ptr_ - amt * num_comps_, num_comps_);
    }
    difference_type operator-(const self_type& rhs) const
    {
      return (ptr_ - rhs.ptr_) / static_cast<difference_type>(num_comps_);
    }
    friend self_type operator+(difference_type lhs, const self_type& rhs)
    {
      return self_type(lhs * rhs.num_comps_ + rhs.ptr_, rhs.num_comps_);
    }
    reference operator[](difference_type amt) const
    {
      return ptr_[amt * num_comps_];
    }
    reference operator*() const
    {
      return *ptr_;
    }
    pointer operator->() const
    {
      return ptr_;
    }
    bool operator==(const self_type& rhs) const
    {
      return ptr_ == rhs.ptr_;
    }
    bool operator!=(const self_type& rhs) const
    {
      return ptr_ != rhs.ptr_;
    }
    bool operator>(const self_type& rhs) const
    {
      return ptr_ > rhs.ptr_;
    }
    bool operator<(const self_type& rhs) const
    {
      return ptr_ < rhs.ptr_;
    }
    bool operator>=(const self_type& rhs) const
    {
      return ptr_ >= rhs.ptr_;
    }
    bool operator<=(const self_type& rhs) const
    {
      return ptr_ <= rhs.ptr_;
    }

    reference comp_value(size_type comp) const
    {
      return *(ptr_ + comp);
    }

  protected:
    const_tuple_iterator(pointer ptr, size_type numComps)
    : ptr_(ptr)
    , num_comps_(numComps)
    {
    }

    friend class EbsdDataArray;

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
    using difference_type = ptrdiff_t;
    using iterator_category = std::random_access_iterator_tag;

    iterator() = delete;

    iterator(const self_type& iter)
    : ptr_(iter.ptr_)
    {
    }

    self_type& operator++()
    {
      ptr_++;
      return *this;
    } // PREFIX
    self_type operator++(int32_t unused)
    {
      // Replace with [[maybe_unused]] in C++ 17
      (void) unused;
      self_type i = *this;
      ptr_++;
      return i;
    } // POSTFIX
    self_type& operator--()
    {
      ptr_--;
      return *this;
    } // PREFIX
    self_type operator--(int32_t unused)
    {
      // Replace with [[maybe_unused]] in C++ 17
      (void)unused;
      self_type i = *this;
      ptr_--;
      return i;
    } // POSTFIX
    self_type& operator+=(difference_type amt)
    {
      ptr_ += amt;
      return *this;
    }
    self_type operator+(difference_type amt) const
    {
      return self_type(ptr_ + amt);
    }
    self_type& operator-=(difference_type amt)
    {
      ptr_ -= amt;
      return *this;
    }
    self_type operator-(difference_type amt) const
    {
      return self_type(ptr_ - amt);
    }
    difference_type operator-(const self_type& rhs) const
    {
      return ptr_ - rhs.ptr_;
    }
    friend self_type operator+(difference_type lhs, const self_type& rhs)
    {
      return self_type(lhs + rhs.ptr_);
    }
    reference operator[](difference_type amt) const
    {
      return ptr_[amt];
    }
    reference operator*() const
    {
      return *ptr_;
    }
    pointer operator->() const
    {
      return ptr_;
    }
    bool operator==(const self_type& rhs) const
    {
      return ptr_ == rhs.ptr_;
    }
    bool operator!=(const self_type& rhs) const
    {
      return ptr_ != rhs.ptr_;
    }
    bool operator>(const self_type& rhs) const
    {
      return ptr_ > rhs.ptr_;
    }
    bool operator<(const self_type& rhs) const
    {
      return ptr_ < rhs.ptr_;
    }
    bool operator>=(const self_type& rhs) const
    {
      return ptr_ >= rhs.ptr_;
    }
    bool operator<=(const self_type& rhs) const
    {
      return ptr_ <= rhs.ptr_;
    }

  protected:
    iterator(pointer ptr)
    : ptr_(ptr)
    {
    }

    friend class EbsdDataArray;

  private:
    pointer ptr_;
  };

  using reverse_iterator = std::reverse_iterator<iterator>;

  class const_iterator
  {
  public:
    using self_type = const_iterator;
    using value_type = T;
    using reference = const T&;
    using pointer = const T*;
    using difference_type = ptrdiff_t;
    using iterator_category = std::random_access_iterator_tag;

    const_iterator() = delete;

    const_iterator(const self_type& iter)
    : ptr_(iter.ptr_)
    {
    }

    self_type& operator++()
    {
      ptr_++;
      return *this;
    } // PREFIX
    self_type operator++(int32_t unused)
    {
      // Replace with [[maybe_unused]] in C++ 17
      (void)unused;
      self_type i = *this;
      ptr_++;
      return i;
    } // POSTFIX
    self_type& operator--()
    {
      ptr_--;
      return *this;
    } // PREFIX
    self_type operator--(int32_t unused)
    {
      // Replace with [[maybe_unused]] in C++ 17
      (void)unused;
      self_type i = *this;
      ptr_--;
      return i;
    } // POSTFIX
    self_type& operator+=(difference_type amt)
    {
      ptr_ += amt;
      return *this;
    }
    self_type operator+(difference_type amt) const
    {
      return self_type(ptr_ + amt);
    }
    self_type& operator-=(difference_type amt)
    {
      ptr_ -= amt;
      return *this;
    }
    self_type operator-(difference_type amt) const
    {
      return self_type(ptr_ - amt);
    }
    difference_type operator-(const self_type& rhs) const
    {
      return ptr_ - rhs.ptr_;
    }
    friend self_type operator+(difference_type lhs, const self_type& rhs)
    {
      return self_type(lhs + rhs.ptr_);
    }
    reference operator[](difference_type amt) const
    {
      return ptr_[amt];
    }
    reference operator*() const
    {
      return *ptr_;
    }
    pointer operator->() const
    {
      return ptr_;
    }
    bool operator==(const self_type& rhs) const
    {
      return ptr_ == rhs.ptr_;
    }
    bool operator!=(const self_type& rhs) const
    {
      return ptr_ != rhs.ptr_;
    }
    bool operator>(const self_type& rhs) const
    {
      return ptr_ > rhs.ptr_;
    }
    bool operator<(const self_type& rhs) const
    {
      return ptr_ < rhs.ptr_;
    }
    bool operator>=(const self_type& rhs) const
    {
      return ptr_ >= rhs.ptr_;
    }
    bool operator<=(const self_type& rhs) const
    {
      return ptr_ <= rhs.ptr_;
    }

  protected:
    const_iterator(pointer ptr)
    : ptr_(ptr)
    {
    }

    friend class EbsdDataArray;

  private:
    pointer ptr_;
  };

  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  // ######### Iterators #########

  tuple_iterator tupleBegin();

  tuple_iterator tupleEnd();

  const_tuple_iterator tupleBegin() const;

  const_tuple_iterator tupleEnd() const;

  const_tuple_iterator constTupleBegin() const;

  const_tuple_iterator constTupleEnd() const;

  iterator begin();

  iterator end();

  const_iterator begin() const;

  const_iterator end() const;

  const_iterator cbegin() const;

  const_iterator cend() const;

  reverse_iterator rbegin();

  reverse_iterator rend();

  const_reverse_iterator rbegin() const;

  const_reverse_iterator rend() const;

  const_reverse_iterator crbegin() const;

  const_reverse_iterator crend() const;

  // ######### Capacity #########

  size_type size() const;

  size_type capacity() const noexcept;
  bool empty() const noexcept;

  // ######### Element Access #########

  inline reference operator[](size_type index)
  {
    return m_Array[index];
  }

  inline const T& operator[](size_type index) const
  {
    return m_Array[index];
  }

  inline reference at(size_type index)
  {
    if(index >= m_Size)
    {
      throw std::out_of_range("EbsdDataArray subscript out of range");
    }
    return m_Array[index];
  }

  inline const T& at(size_type index) const
  {
    if(index >= m_Size)
    {
      throw std::out_of_range("EbsdDataArray subscript out of range");
    }
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
  void assign(size_type n, const value_type& val);

  /**
   * @brief In the initializer list version (3), the new contents are copies of the values passed as initializer list, in the same order.
   * @param il
   */
  void assign(std::initializer_list<value_type> il);

  /**
   * @brief push_back
   * @param val
   */
  void push_back(const value_type& val);
  /**
   * @brief push_back
   * @param val
   */
  void push_back(value_type&& val);

  /**
   * @brief pop_back
   */
  void pop_back();

  /**
   * @brief Removes all elements from the array (which are destroyed), leaving the container with a size of 0.
   */
  void clear();

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
  void deallocate();

  /**
   * @brief Resizes the internal array
   * @param size The new size of the internal array
   * @return 1 on success, 0 on failure
   */
  int32_t resizeTotalElements(size_t size);

  /**
   * @brief resizes the internal array to be 'size' elements in length
   * @param size
   * @return Pointer to the internal array
   */
  T* resizeAndExtend(size_t size);

private:
  QString m_Name = {};
  T* m_Array = nullptr;
  size_t m_Size = 0;
  size_t m_MaxId = 0;
  size_t m_NumTuples = 0;
  size_t m_NumComponents = 1;
  T m_InitValue = static_cast<T>(0);
  comp_dims_type m_CompDims = {1};
  bool m_IsAllocated = false;
  bool m_OwnsData = true;
};

// -----------------------------------------------------------------------------
// Declare our extern templates
extern template class EbsdDataArray<bool>;

extern template class EbsdDataArray<char>;
extern template class EbsdDataArray<unsigned char>;

extern template class EbsdDataArray<int8_t>;
extern template class EbsdDataArray<uint8_t>;
extern template class EbsdDataArray<int16_t>;
extern template class EbsdDataArray<uint16_t>;
extern template class EbsdDataArray<int32_t>;
extern template class EbsdDataArray<uint32_t>;
extern template class EbsdDataArray<int64_t>;
extern template class EbsdDataArray<uint64_t>;

extern template class EbsdDataArray<float>;
extern template class EbsdDataArray<double>;

extern template class EbsdDataArray<size_t>;

// -----------------------------------------------------------------------------
// Declare our aliases
namespace EbsdLib
{
// using BoolArrayType = EbsdDataArray<bool>;

// using CharArrayType = EbsdDataArray<char>;
// using UCharArrayType = EbsdDataArray<unsigned char>;

// using Int8ArrayType = EbsdDataArray<int8_t>;
using UInt8ArrayType = EbsdDataArray<uint8_t>;

// using Int16ArrayType = EbsdDataArray<int16_t>;
// using UInt16ArrayType = EbsdDataArray<uint16_t>;

using Int32ArrayType = EbsdDataArray<int32_t>;
// using UInt32ArrayType = EbsdDataArray<uint32_t>;

// using Int64ArrayType = EbsdDataArray<int64_t>;
// using UInt64ArrayType = EbsdDataArray<uint64_t>;

using FloatArrayType = EbsdDataArray<float>;
using DoubleArrayType = EbsdDataArray<double>;

// using SizeTArrayType = EbsdDataArray<size_t>;
} // namespace EbsdLib
