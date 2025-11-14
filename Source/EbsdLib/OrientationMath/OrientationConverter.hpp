/* ============================================================================
 * Copyright (c) 2009-2017 BlueQuartz Software, LLC
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

#include <iostream>
#include <memory>
#include <vector>

#include <string>

#include "EbsdLib/Core/EbsdSetGetMacros.h"
#include "EbsdLib/Orientation/AxisAngle.hpp"
#include "EbsdLib/Orientation/Cubochoric.hpp"
#include "EbsdLib/Orientation/Euler.hpp"
#include "EbsdLib/Orientation/Homochoric.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Orientation/OrientationMatrix.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"
#include "EbsdLib/Orientation/Stereographic.hpp"

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Math/EbsdLibMath.h"

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>
#endif

#define OC_CLASS_DEFINES(name)                                                                                                                                                                         \
  using DataArrayPointerType = typename DataArrayType::Pointer;                                                                                                                                        \
  using Self = name<DataArrayType, T>;                                                                                                                                                                 \
  using Pointer = std::shared_ptr<Self>;                                                                                                                                                               \
  static Pointer New()                                                                                                                                                                                 \
  {                                                                                                                                                                                                    \
    Pointer sharedPtr(new(Self));                                                                                                                                                                      \
    return sharedPtr;                                                                                                                                                                                  \
  }                                                                                                                                                                                                    \
  std::string getNameOfClass() const override                                                                                                                                                          \
  {                                                                                                                                                                                                    \
    return std::string(#name);                                                                                                                                                                         \
  }

namespace ebsdlib
{
/**
 * @brief This is the top level superclass for doing the conversions between orientation
 * representations
 */
template <class DataArrayType, typename T>
class OrientationConverter
{
public:
  using DataArrayPointerType = typename DataArrayType::Pointer;
  using Self = OrientationConverter<DataArrayType, T>;
  using Pointer = std::shared_ptr<Self>;

  virtual ~OrientationConverter() = default;

  virtual std::string getNameOfClass() const
  {
    return std::string("OrientationConverter<DataArrayType,T>");
  }

  /**
   * @brief getOrientationRepresentation
   * @return
   */
  ebsdlib::orientations::Type getOrientationRepresentation()
  {
    return ebsdlib::orientations::Type::Unknown;
  }

  /**
   * @brief convertRepresentationTo Converts the data to the desired type
   * @param repType The type of representation to convert to.
   * @return
   */
  void convertRepresentationTo(ebsdlib::orientations::Type repType)
  {
    if(repType == ebsdlib::orientations::Type::Euler)
    {
      toEulers();
    }
    else if(repType == ebsdlib::orientations::Type::OrientationMatrix)
    {
      toOrientationMatrix();
    }
    else if(repType == ebsdlib::orientations::Type::Quaternion)
    {
      toQuaternion();
    }
    else if(repType == ebsdlib::orientations::Type::AxisAngle)
    {
      toAxisAngle();
    }
    else if(repType == ebsdlib::orientations::Type::Rodrigues)
    {
      toRodrigues();
    }
    else if(repType == ebsdlib::orientations::Type::Homochoric)
    {
      toHomochoric();
    }
    else if(repType == ebsdlib::orientations::Type::Cubochoric)
    {
      toCubochoric();
    }
    else if(repType == ebsdlib::orientations::Type::Stereographic)
    {
      toStereographic();
    }
  }

  /**
   * @brief toEulers Converts the input orientations to Euler Angles
   */
  virtual void toEulers() = 0;

  /**
   * @brief toOrientationMatrix  Converts the input orientations to an Orientation Matrix (3x3)
   */
  virtual void toOrientationMatrix() = 0;

  /**
   * @brief toQuaternion  Converts the input orientations to Quaternions
   */
  virtual void toQuaternion() = 0;

  /**
   * @brief toAxisAngle  Converts the input orientations to Axis Angles
   */
  virtual void toAxisAngle() = 0;

  /**
   * @brief toRodrigues  Converts the input orientations to Rodrigues
   */
  virtual void toRodrigues() = 0;

  /**
   * @brief toHomochoric  Converts the input orientations to Homochoric
   */
  virtual void toHomochoric() = 0;

  /**
   * @brief toCubochoric  Converts the input orientations to Cubochoric
   */
  virtual void toCubochoric() = 0;

  /**
   * @brief toStereographic Converts the input orientations to Stereographic
   *
   */
  virtual void toStereographic() = 0;

  /**
   * @brief sanityCheckInputData Runs basic checks on the input data to ensure
   * the intput data falls within certain data ranges.
   */
  virtual void sanityCheckInputData() = 0;

  /**
   * @brief printRepresentation Prints the values of a single representation to
   * an output stream;
   * @param out An output stream
   * @param a Pointer to print
   * @param label Optional Label
   */
  //   void printRepresentation(std::ostream& out, T* a, const std::string& label = std::string("")) = 0;

  /**
   * @brief Sets/Gets the input orientations
   */
  void setInputData(DataArrayPointerType& input)
  {
    m_InputData = input;
  }
  DataArrayPointerType getInputData() const
  {
    return m_InputData;
  }

  /**
   * @brief Sets/Gets the output orientations
   */
  void setOutputData(DataArrayPointerType& output)
  {
    m_OutputData = output;
  }
  DataArrayPointerType getOutputData() const
  {
    return m_OutputData;
  }

  /**
   * @brief GetOrientationTypeStrings
   * @return
   */
  template <typename ContainerType>
  static ContainerType GetOrientationTypeStrings()
  {
    ContainerType otypes(8);
    otypes[0] = "Euler";
    otypes[1] = "Orientation Matrix";
    otypes[2] = "Quaternion";
    otypes[3] = "Axis-Angle";
    otypes[4] = "Rodrigues";
    otypes[5] = "Homochoric";
    otypes[6] = "Cubochoric";
    otypes[7] = "Stereographic";
    return otypes;
  }

  /**
   * @brief GetComponentCounts
   * @return
   */
  template <typename ContainerType>
  static ContainerType GetComponentCounts()
  {
    ContainerType counts(8);
    counts[0] = 3; // Euler
    counts[1] = 9; // Orientation Matrix
    counts[2] = 4; // Quaternion
    counts[3] = 4; // Axis-Angle
    counts[4] = 4; // Rodrigues
    counts[5] = 3; // Homchoric
    counts[6] = 3; // Cubochoric
    counts[7] = 3; // Stereographic
    return counts;
  }

  /**
   * @brief GetOrientationTypes
   * @return
   */
  static std::vector<ebsdlib::orientations::Type> GetOrientationTypes()
  {
    std::vector<ebsdlib::orientations::Type> ocTypes(8);
    ocTypes[0] = ebsdlib::orientations::Type::Euler;
    ocTypes[1] = ebsdlib::orientations::Type::OrientationMatrix;
    ocTypes[2] = ebsdlib::orientations::Type::Quaternion;
    ocTypes[3] = ebsdlib::orientations::Type::AxisAngle;
    ocTypes[4] = ebsdlib::orientations::Type::Rodrigues;
    ocTypes[5] = ebsdlib::orientations::Type::Homochoric;
    ocTypes[6] = ebsdlib::orientations::Type::Cubochoric;
    ocTypes[7] = ebsdlib::orientations::Type::Stereographic;
    return ocTypes;
  }

  /**
   * @brief GetMinIndex
   * @return
   */
  static int GetMinIndex()
  {
    return 0;
  }

  /**
   * @brief Returns the maximum index into the arrays of conversion types and representations
   * @return
   */
  static int GetMaxIndex()
  {
    return static_cast<int>(static_cast<int>(ebsdlib::orientations::Type::Unknown) - 1);
  }

protected:
  OrientationConverter() = default;

public:
  OrientationConverter(const OrientationConverter&) = delete;            // Copy Constructor Not Implemented
  OrientationConverter(OrientationConverter&&) = delete;                 // Move Constructor Not Implemented
  OrientationConverter& operator=(const OrientationConverter&) = delete; // Copy Assignment Not Implemented
  OrientationConverter& operator=(OrientationConverter&&) = delete;      // Move Assignment Not Implemented

private:
  DataArrayPointerType m_InputData;
  DataArrayPointerType m_OutputData;
};

/**
 * @brief This templated class is a functor class that is used for
 * the TBB classes to use to parallelize the conversion of orientation
 * representations
 */
template <typename T, class Converter>
class ConvertRepresentation
{
public:
  ConvertRepresentation(T* inPtr, T* outPtr, size_t inStride, size_t outStride)
  : m_InPtr(inPtr)
  , m_OutPtr(outPtr)
  , m_InStride(inStride)
  , m_OutStride(outStride)
  {
  }
  virtual ~ConvertRepresentation() = default;

  /**
   * @brief This is the main conversion routine
   * @param start Starting index
   * @param end Ending index
   */
  void convert(size_t start, size_t end) const
  {
    Converter conv;
    T* input = m_InPtr + (start * m_InStride);
    T* output = m_OutPtr + (start * m_OutStride);
    for(size_t i = start; i < end; ++i)
    {
      conv(input, output);
      input = input + m_InStride;    /* Increment input pointer */
      output = output + m_OutStride; /* Increment output pointer*/
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    convert(r.begin(), r.end());
  }
#endif

private:
  T* m_InPtr = nullptr;
  T* m_OutPtr = nullptr;
  size_t m_InStride = 0;
  size_t m_OutStride = 0;
};

#define OC_TBB_IMPL(TO_REP)                                                                                                                                                                            \
  template <typename T, class InputType, class OutputType>                                                                                                                                             \
  class to##TO_REP##Convertor                                                                                                                                                                          \
  {                                                                                                                                                                                                    \
  public:                                                                                                                                                                                              \
    to##TO_REP##Convertor(T* inputPtr, T* outputPtr)                                                                                                                                                   \
    : m_Input(inputPtr)                                                                                                                                                                                \
    , m_Output(outputPtr)                                                                                                                                                                              \
    {                                                                                                                                                                                                  \
    }                                                                                                                                                                                                  \
    void operator()(const tbb::blocked_range<size_t>& r) const                                                                                                                                         \
    {                                                                                                                                                                                                  \
      InputType inputInstance;                                                                                                                                                                         \
      OutputType outputInstance;                                                                                                                                                                       \
      size_t inStride = inputInstance.size();                                                                                                                                                          \
      size_t outStride = outputInstance.size();                                                                                                                                                        \
      for(size_t i = r.begin(); i < r.end(); ++i)                                                                                                                                                      \
      {                                                                                                                                                                                                \
        size_t inOffset = i * inStride;                                                                                                                                                                \
        size_t outOffset = i * outStride;                                                                                                                                                              \
        inputInstance = InputType(m_Input + inOffset);                                                                                                                                                 \
        outputInstance = inputInstance.to##TO_REP();                                                                                                                                                   \
        outputInstance.copyTo(m_Output + outOffset);                                                                                                                                                   \
      }                                                                                                                                                                                                \
    }                                                                                                                                                                                                  \
                                                                                                                                                                                                       \
  private:                                                                                                                                                                                             \
    T* m_Input = nullptr;                                                                                                                                                                              \
    T* m_Output = nullptr;                                                                                                                                                                             \
  };

OC_TBB_IMPL(Euler)
// OC_TBB_IMPL(OrientationMatrix)
template <typename T, class InputType, class OutputType>
class toOrientationMatrixConvertor
{
public:
  toOrientationMatrixConvertor(T* inputPtr, T* outputPtr)
  : m_Input(inputPtr)
  , m_Output(outputPtr)
  {
  }

  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    InputType inputInstance;
    OutputType outputInstance;
    size_t inStride = inputInstance.size();
    size_t outStride = outputInstance.size();
    for(size_t i = r.begin(); i < r.end(); ++i)
    {
      size_t inOffset = i * inStride;
      size_t outOffset = i * outStride;
      InputType inputInstance2(m_Input + inOffset);
      outputInstance = inputInstance2.toOrientationMatrix();
      outputInstance.copyTo(m_Output + outOffset);
    }
  }

private:
  T* m_Input = nullptr;
  T* m_Output = nullptr;
};

OC_TBB_IMPL(Quaternion)
OC_TBB_IMPL(AxisAngle)
OC_TBB_IMPL(Rodrigues)
OC_TBB_IMPL(Homochoric)
OC_TBB_IMPL(Cubochoric)
OC_TBB_IMPL(Stereographic)

#define OC_CONVERT_BODY_PREAMBLE(FROM_REP, TO_REP)                                                                                                                                                     \
  sanityCheckInputData();                                                                                                                                                                              \
  DataArrayPointerType input = this->getInputData();                                                                                                                                                   \
  T* inPtr = input->getPointer(0);                                                                                                                                                                     \
  size_t nTuples = this->getInputData()->getNumberOfTuples();                                                                                                                                          \
  ebsdlib::TO_REP<T> outputInstance;                                                                                                                                                                   \
  size_t outStride = outputInstance.size();                                                                                                                                                            \
  std::vector<size_t> cDims = {outStride};                                                                                                                                                             \
  DataArrayPointerType output = DataArrayType::CreateArray(nTuples, cDims, #TO_REP, true);                                                                                                             \
  output->initializeWithZeros(); /* Initialize the array with Zeros */                                                                                                                                 \
  T* outPtr = output->getPointer(0);                                                                                                                                                                   \
  using FROM_REP##Type = ebsdlib::FROM_REP<T>;                                                                                                                                                         \
  using TO_REP##Type = ebsdlib::TO_REP<T>;                                                                                                                                                             \
  tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), to##TO_REP##Convertor<T, FROM_REP##Type, TO_REP##Type>(inPtr, outPtr), tbb::auto_partitioner());                                           \
  this->setOutputData(output);

/* =============================================================================
 *
 * ===========================================================================*/

template <typename T>
class EulerSanityCheck
{
public:
  EulerSanityCheck(T* input, size_t stride)
  : m_Input(input)
  , m_Stride(stride)
  {
  }
  virtual ~EulerSanityCheck() = default;

  void sanityCheck(size_t start, size_t end) const
  {
    T* inPtr = m_Input + (start * m_Stride);

    for(size_t i = start; i < end; ++i)
    {
      inPtr[0] = static_cast<T>(std::fmod(inPtr[0], ebsdlib::constants::k_2PiD));
      inPtr[1] = static_cast<T>(std::fmod(inPtr[1], ebsdlib::constants::k_PiD));
      inPtr[2] = static_cast<T>(std::fmod(inPtr[2], ebsdlib::constants::k_2PiD));

      if(inPtr[0] < 0.0)
      {
        inPtr[0] *= static_cast<T>(-1.0);
      }
      if(inPtr[1] < 0.0)
      {
        inPtr[1] *= static_cast<T>(-1.0);
      }
      if(inPtr[2] < 0.0)
      {
        inPtr[2] *= static_cast<T>(-1.0);
      }

      inPtr = inPtr + m_Stride; // This is Pointer arithmetic!!
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    sanityCheck(r.begin(), r.end());
  }
#endif

private:
  T* m_Input = nullptr;
  size_t m_Stride = 0;
};

// -----------------------------------------------------------------------------
template <class DataArrayType, typename T>
class EulerConverter : public OrientationConverter<DataArrayType, T>
{
public:
  OC_CLASS_DEFINES(EulerConverter)

  ~EulerConverter() override = default;

  ebsdlib::orientations::Type getOrientationRepresentation()
  {
    return ebsdlib::orientations::Type::Euler;
  }

  void toEulers() override
  {
    DataArrayPointerType input = this->getInputData();
    DataArrayPointerType output = std::dynamic_pointer_cast<DataArrayType>(input->deepCopy());
    this->setOutputData(output);
  }

  void toOrientationMatrix() override
  {
    // OC_CONVERT_BODY_PREAMBLE(Euler, OrientationMatrix);
    sanityCheckInputData();
    DataArrayPointerType input = this->getInputData();
    T* inPtr = input->getPointer(0); /* Get a Raw pointer to the chunk of memory */
    size_t nTuples = this->getInputData()->getNumberOfTuples();
    ebsdlib::OrientationMatrix<T> outputInstance;
    size_t outStride = outputInstance.size();
    std::vector<size_t> cDims = {outStride};
    DataArrayPointerType output = DataArrayType::CreateArray(nTuples, cDims, "OrientationMatrix", true);
    output->initializeWithZeros();
    T* outPtr = output->getPointer(0);
    using EulerType = ebsdlib::Euler<T>;
    using OrientationMatrixType = ebsdlib::OrientationMatrix<T>;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), toOrientationMatrixConvertor<T, Euler<T>, OrientationMatrix<T>>(inPtr, outPtr), tbb::auto_partitioner());
    this->setOutputData(output);
    ;
  }

  void toQuaternion() override
  {
    OC_CONVERT_BODY_PREAMBLE(Euler, Quaternion);
  }

  void toAxisAngle() override
  {
    OC_CONVERT_BODY_PREAMBLE(Euler, AxisAngle);
  }

  void toRodrigues() override
  {
    OC_CONVERT_BODY_PREAMBLE(Euler, Rodrigues);
  }

  void toHomochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Euler, Homochoric)
  }

  void toCubochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Euler, Cubochoric)
  }

  void toStereographic() override
  {
    OC_CONVERT_BODY_PREAMBLE(Euler, Stereographic)
  }

  void sanityCheckInputData() override
  {
    DataArrayPointerType input = this->getInputData();
    T* inPtr = input->getPointer(0);
    size_t nTuples = input->getNumberOfTuples();
    int inStride = input->getNumberOfComponents();

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
    bool doParallel = true;
    if(doParallel)
    {
      tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), EulerSanityCheck<T>(inPtr, inStride), tbb::auto_partitioner());
    }
    else
#endif
    {
      EulerSanityCheck<T> serial(inPtr, inStride);
      serial.sanityCheck(0, nTuples);
    }
  }

  /**
   * @brief compareRepresentations
   * @param a
   * @param b
   * @param epsilon
   * @return
   */
  bool compareRepresentations(T* a, T* b, const T& epsilon = std::numeric_limits<T>::epsilon())
  {
    bool close = false;
    for(int i = 0; i < 3; i++)
    {
      close = (epsilon > std::fabs(a[i] - b[i]));
      if(!close)
      {
        return close;
      }
    }
    return close;
  }

  /**
   * @brief printRepresentation
   * @param out
   * @param eu
   * @param label
   */
  void printRepresentation(std::ostream& out, T* eu, const std::string& label = std::string("Eu"))
  {
    out.precision(16);
    out << label << eu[0] << '\t' << eu[1] << '\t' << eu[2] << std::endl;
  }

protected:
  EulerConverter()
  : OrientationConverter<DataArrayType, T>()
  {
  }

  explicit EulerConverter(DataArrayPointerType data)
  : OrientationConverter<DataArrayType, T>()
  {
    this->setInputData(data);
  }

public:
  EulerConverter(const EulerConverter&) = delete;            // Copy Constructor Not Implemented
  EulerConverter(EulerConverter&&) = delete;                 // Move Constructor Not Implemented
  EulerConverter& operator=(const EulerConverter&) = delete; // Copy Assignment Not Implemented
  EulerConverter& operator=(EulerConverter&&) = delete;      // Move Assignment Not Implemented
};

/* =============================================================================
 *
 * ===========================================================================*/
template <typename T>
class OrientationMatrixSanityCheck
{
public:
  OrientationMatrixSanityCheck(T* input, size_t stride)
  : m_Input(input)
  , m_Stride(stride)
  {
  }
  virtual ~OrientationMatrixSanityCheck() = default;

  void sanityCheck(size_t start, size_t end) const
  {
    T* inPtr = m_Input + (start * m_Stride);

    for(size_t i = start; i < end; ++i)
    {
      using Orientation_Type = ebsdlib::OrientationMatrix<T>;

      Orientation_Type oaType(inPtr);

      auto res = oaType.isValid();
      if(res.result <= 0)
      {
        std::cout << res.msg << std::endl;
        printRepresentation(std::cout, inPtr, std::string("Bad OM"));
      }

      inPtr = inPtr + m_Stride; // This is Pointer arithmetic!!
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    sanityCheck(r.begin(), r.end());
  }
#endif

  /**
   * @brief printRepresentation
   * @param out
   * @param om
   * @param label
   */
  void printRepresentation(std::ostream& out, T* om, const std::string& label = std::string("Om")) const
  {
    out.precision(16);
    out << label << om[0] << '\t' << om[1] << '\t' << om[2] << std::endl;
    out << label << om[3] << '\t' << om[4] << '\t' << om[5] << std::endl;
    out << label << om[6] << '\t' << om[7] << '\t' << om[8] << std::endl;
  }

private:
  T* m_Input = nullptr;
  size_t m_Stride = 0;
};

template <class DataArrayType, typename T>
class OrientationMatrixConverter : public OrientationConverter<DataArrayType, T>
{
public:
  OC_CLASS_DEFINES(OrientationMatrixConverter)

  ~OrientationMatrixConverter() override = default;

  ebsdlib::orientations::Type getOrientationRepresentation()
  {
    return ebsdlib::orientations::Type::OrientationMatrix;
  }

  void toEulers() override
  {
    OC_CONVERT_BODY_PREAMBLE(OrientationMatrix, Euler);
  }

  void toOrientationMatrix() override
  {
    DataArrayPointerType input = this->getInputData();
    DataArrayPointerType output = std::dynamic_pointer_cast<DataArrayType>(input->deepCopy());
    this->setOutputData(output);
  }

  void toQuaternion() override
  {
    sanityCheckInputData();
    OC_CONVERT_BODY_PREAMBLE(OrientationMatrix, Quaternion);
  }

  void toAxisAngle() override
  {
    sanityCheckInputData();
    OC_CONVERT_BODY_PREAMBLE(OrientationMatrix, AxisAngle);
  }

  void toRodrigues() override
  {
    sanityCheckInputData();
    OC_CONVERT_BODY_PREAMBLE(OrientationMatrix, Rodrigues);
  }

  void toHomochoric() override
  {
    sanityCheckInputData();
    OC_CONVERT_BODY_PREAMBLE(OrientationMatrix, Homochoric)
  }

  void toCubochoric() override
  {
    sanityCheckInputData();
    OC_CONVERT_BODY_PREAMBLE(OrientationMatrix, Cubochoric)
  }

  void toStereographic() override
  {
    OC_CONVERT_BODY_PREAMBLE(OrientationMatrix, Stereographic)
  }

  void sanityCheckInputData() override
  {
    DataArrayPointerType input = this->getInputData();
    T* inPtr = input->getPointer(0);
    size_t nTuples = input->getNumberOfTuples();
    int inStride = input->getNumberOfComponents();

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
    bool doParallel = true;
    if(doParallel)
    {
      tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), OrientationMatrixSanityCheck<T>(inPtr, inStride), tbb::auto_partitioner());
    }
    else
#endif
    {
      OrientationMatrixSanityCheck<T> serial(inPtr, inStride);
      serial.sanityCheck(0, nTuples);
    }
  }

  /**
   * @brief compareRepresentations
   * @param a
   * @param b
   * @param epsilon
   * @return
   */
  bool compareRepresentations(T* a, T* b, const T& epsilon = std::numeric_limits<T>::epsilon())
  {
    bool close = false;
    for(int i = 0; i < 9; i++)
    {
      close = (epsilon > std::fabs(a[i] - b[i]));
      if(!close)
      {
        return close;
      }
    }
    return close;
  }

  /**
   * @brief printRepresentation
   * @param out
   * @param om
   * @param label
   */
  void printRepresentation(std::ostream& out, T* om, const std::string& label = std::string("Om"))
  {
    out.precision(16);
    out << label << om[0] << '\t' << om[1] << '\t' << om[2] << std::endl;
    out << label << om[3] << '\t' << om[4] << '\t' << om[5] << std::endl;
    out << label << om[6] << '\t' << om[7] << '\t' << om[8] << std::endl;
  }

protected:
  OrientationMatrixConverter()
  : OrientationConverter<DataArrayType, T>()
  {
  }
  explicit OrientationMatrixConverter(DataArrayPointerType data)
  : OrientationConverter<DataArrayType, T>()
  {
    this->setInputData(data);
  }

public:
  OrientationMatrixConverter(const OrientationMatrixConverter&) = delete;            // Copy Constructor Not Implemented
  OrientationMatrixConverter(OrientationMatrixConverter&&) = delete;                 // Move Constructor Not Implemented
  OrientationMatrixConverter& operator=(const OrientationMatrixConverter&) = delete; // Copy Assignment Not Implemented
  OrientationMatrixConverter& operator=(OrientationMatrixConverter&&) = delete;      // Move Assignment Not Implemented
};

/* =============================================================================
 *
 * ===========================================================================*/

template <typename T>
class QuaternionSanityCheck
{
public:
  QuaternionSanityCheck(T* input, size_t stride)
  : m_Input(input)
  , m_Stride(stride)
  {
  }
  virtual ~QuaternionSanityCheck() = default;

  void sanityCheck(size_t start, size_t end) const
  {
    T* inPtr = m_Input + (start * m_Stride);

    for(size_t i = start; i < end; ++i)
    {
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    sanityCheck(r.begin(), r.end());
  }
#endif

private:
  T* m_Input = nullptr;
  size_t m_Stride = 0;
};

template <class DataArrayType, typename T>
class QuaternionConverter : public OrientationConverter<DataArrayType, T>
{
public:
  OC_CLASS_DEFINES(QuaternionConverter)

  ~QuaternionConverter() override = default;

  ebsdlib::orientations::Type getOrientationRepresentation()
  {
    return ebsdlib::orientations::Type::Quaternion;
  }

  void toEulers() override
  {
    OC_CONVERT_BODY_PREAMBLE(Quaternion, Euler);
  }

  void toOrientationMatrix() override
  {
    OC_CONVERT_BODY_PREAMBLE(Quaternion, OrientationMatrix);
  }

  void toQuaternion() override
  {
    using PointerType = DataArrayPointerType;
    PointerType input = this->getInputData();
    PointerType output = std::dynamic_pointer_cast<DataArrayType>(input->deepCopy());
    this->setOutputData(output);
  }

  void toAxisAngle() override
  {
    OC_CONVERT_BODY_PREAMBLE(Quaternion, AxisAngle);
  }

  void toRodrigues() override
  {
    OC_CONVERT_BODY_PREAMBLE(Quaternion, Rodrigues);
  }

  void toHomochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Quaternion, Homochoric)
  }

  void toCubochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Quaternion, Cubochoric)
  }

  void toStereographic() override
  {
    OC_CONVERT_BODY_PREAMBLE(Quaternion, Stereographic)
  }

  void sanityCheckInputData() override
  {
    /* Apparently there is no sanity check for Quaternions, Odd. We place this
     * code here in case we come up with one, the parallel version is ready to
     * go
     */
#if 0
    DataArrayPointerType input = this->getInputData();
    T* inPtr = input->getPointer(0);
    size_t nTuples = input->getNumberOfTuples();
    int inStride = input->getNumberOfComponents();

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), QuaternionSanityCheck<T>(inPtr, inStride), tbb::auto_partitioner());
#else
    QuaternionSanityCheck<T> serial(inPtr, inStride);
    serial.sanityCheck(0, nTuples);
#endif
#endif
  }

  /**
   * @brief printRepresentation
   * @param out
   * @param qu
   * @param label
   */
  void printRepresentation(std::ostream& out, T* qu, const std::string& label = std::string("Qu"))
  {
    out.precision(16);
    out << label << qu[0] << '\t' << qu[1] << '\t' << qu[2] << '\t' << qu[3] << std::endl;
  }

  /**
   * @brief compareRepresentations
   * @param a
   * @param b
   * @param epsilon
   * @return
   */
  bool compareRepresentations(T* a, T* b, const T& epsilon = std::numeric_limits<T>::epsilon())
  {
    bool close = false;
    for(int i = 0; i < 4; i++)
    {
      close = (epsilon > std::fabs(a[i] - b[i]));
      if(!close)
      {
        return close;
      }
    }
    return close;
  }

protected:
  QuaternionConverter()
  : OrientationConverter<DataArrayType, T>()
  {
  }
  explicit QuaternionConverter(DataArrayPointerType data)
  : OrientationConverter<DataArrayType, T>()
  {
    this->setInputData(data);
  }

public:
  QuaternionConverter(const QuaternionConverter&) = delete;            // Copy Constructor Not Implemented
  QuaternionConverter(QuaternionConverter&&) = delete;                 // Move Constructor Not Implemented
  QuaternionConverter& operator=(const QuaternionConverter&) = delete; // Copy Assignment Not Implemented
  QuaternionConverter& operator=(QuaternionConverter&&) = delete;      // Move Assignment Not Implemented
};

/* =============================================================================
 *
 * ===========================================================================*/
template <typename T>
class AxisAngleSanityCheck
{
public:
  AxisAngleSanityCheck(T* input, size_t stride)
  : m_Input(input)
  , m_Stride(stride)
  {
  }
  virtual ~AxisAngleSanityCheck() = default;

  void sanityCheck(size_t start, size_t end) const
  {
    T* inPtr = m_Input + (start * m_Stride);

    for(size_t i = start; i < end; ++i)
    {
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    sanityCheck(r.begin(), r.end());
  }
#endif

private:
  T* m_Input = nullptr;
  size_t m_Stride = 0;
};

template <class DataArrayType, typename T>
class AxisAngleConverter : public OrientationConverter<DataArrayType, T>
{
public:
  OC_CLASS_DEFINES(AxisAngleConverter)

  ~AxisAngleConverter() override = default;

  ebsdlib::orientations::Type getOrientationRepresentation()
  {
    return ebsdlib::orientations::Type::AxisAngle;
  }

  void toEulers() override
  {
    OC_CONVERT_BODY_PREAMBLE(AxisAngle, Euler);
  }

  void toOrientationMatrix() override
  {
    OC_CONVERT_BODY_PREAMBLE(AxisAngle, OrientationMatrix);
  }

  void toQuaternion() override
  {
    OC_CONVERT_BODY_PREAMBLE(AxisAngle, Quaternion);
  }

  void toAxisAngle() override
  {
    using PointerType = DataArrayPointerType;
    PointerType input = this->getInputData();
    PointerType output = std::dynamic_pointer_cast<DataArrayType>(input->deepCopy());
    this->setOutputData(output);
  }

  void toRodrigues() override
  {
    OC_CONVERT_BODY_PREAMBLE(AxisAngle, Rodrigues);
  }

  void toHomochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(AxisAngle, Homochoric)
  }

  void toCubochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(AxisAngle, Cubochoric)
  }

  void toStereographic() override
  {
    OC_CONVERT_BODY_PREAMBLE(AxisAngle, Stereographic)
  }

  void sanityCheckInputData() override
  {
    /* Apparently there is no sanity check for AxisAngle, Odd. We place this
     * code here in case we come up with one, the parallel version is ready to
     * go
     */
#if 0
    DataArrayPointerType input = this->getInputData();
    T* inPtr = input->getPointer(0);
    size_t nTuples = input->getNumberOfTuples();
    int inStride = input->getNumberOfComponents();
#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), AxisAngleSanityCheck<T>(inPtr, inStride), tbb::auto_partitioner());
#else
    AxisAngleSanityCheck<T> serial(inPtr, inStride);
    serial.sanityCheck(0, nTuples);
#endif
#endif
  }

  /**
   * @brief compareRepresentations
   * @param a
   * @param b
   * @param epsilon
   * @return
   */
  bool compareRepresentations(T* a, T* b, const T& epsilon = std::numeric_limits<T>::epsilon())
  {
    bool close = false;
    for(int i = 0; i < 4; i++)
    {
      close = (epsilon > std::fabs(a[i] - b[i]));
      if(!close)
      {
        return close;
      }
    }
    return close;
  }

  /**
   * @brief printRepresentation
   * @param out
   * @param ax
   * @param label
   */
  void printRepresentation(std::ostream& out, T* ax, const std::string& label = std::string("Ax"))
  {
    out.precision(16);
    out << label << "<" << ax[0] << '\t' << ax[1] << '\t' << ax[2] << ">\t" << ax[3] << std::endl;
  }

protected:
  AxisAngleConverter()
  : OrientationConverter<DataArrayType, T>()
  {
  }

  explicit AxisAngleConverter(DataArrayPointerType data)
  : OrientationConverter<DataArrayType, T>()
  {
    this->setInputData(data);
  }

public:
  AxisAngleConverter(const AxisAngleConverter&) = delete;            // Copy Constructor Not Implemented
  AxisAngleConverter(AxisAngleConverter&&) = delete;                 // Move Constructor Not Implemented
  AxisAngleConverter& operator=(const AxisAngleConverter&) = delete; // Copy Assignment Not Implemented
  AxisAngleConverter& operator=(AxisAngleConverter&&) = delete;      // Move Assignment Not Implemented
};

/* =============================================================================
 *
 * ===========================================================================*/

template <typename T>
class RodriguesSanityCheck
{
public:
  RodriguesSanityCheck(T* input, size_t stride)
  : m_Input(input)
  , m_Stride(stride)
  {
  }
  virtual ~RodriguesSanityCheck() = default;

  void sanityCheck(size_t start, size_t end) const
  {
    T* inPtr = m_Input + (start * m_Stride);

    for(size_t i = start; i < end; ++i)
    {
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    sanityCheck(r.begin(), r.end());
  }
#endif

private:
  T* m_Input = nullptr;
  size_t m_Stride = 0;
};

template <class DataArrayType, typename T>
class RodriguesConverter : public OrientationConverter<DataArrayType, T>
{
public:
  OC_CLASS_DEFINES(RodriguesConverter)

  ~RodriguesConverter() override = default;

  ebsdlib::orientations::Type getOrientationRepresentation()
  {
    return ebsdlib::orientations::Type::Rodrigues;
  }

  void toEulers() override
  {
    OC_CONVERT_BODY_PREAMBLE(Rodrigues, Euler);
  }

  void toOrientationMatrix() override
  {
    OC_CONVERT_BODY_PREAMBLE(Rodrigues, OrientationMatrix);
  }

  void toQuaternion() override
  {
    OC_CONVERT_BODY_PREAMBLE(Rodrigues, Quaternion);
  }

  void toAxisAngle() override
  {
    OC_CONVERT_BODY_PREAMBLE(Rodrigues, AxisAngle);
  }

  void toRodrigues() override
  {
    using PointerType = DataArrayPointerType;
    PointerType input = this->getInputData();
    PointerType output = std::dynamic_pointer_cast<DataArrayType>(input->deepCopy());
    this->setOutputData(output);
  }

  void toHomochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Rodrigues, Homochoric)
  }

  void toCubochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Rodrigues, Cubochoric)
  }

  void toStereographic() override
  {
    OC_CONVERT_BODY_PREAMBLE(Rodrigues, Stereographic)
  }

  void sanityCheckInputData() override
  {
    /* Apparently there is no sanity check for Rodrigues, Odd. We place this
     * code here in case we come up with one, the parallel version is ready to
     * go
     */
#if 0
    DataArrayPointerType input = this->getInputData();
    T* inPtr = input->getPointer(0);
    size_t nTuples = input->getNumberOfTuples();
    int inStride = input->getNumberOfComponents();
#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), RodriguesSanityCheck<DataArrayType>(inPtr, inStride), tbb::auto_partitioner());
#else
    RodriguesSanityCheck<T> serial(inPtr, inStride);
    serial.sanityCheck(0, nTuples);
#endif
#endif
  }

  /**
   * @brief compareRepresentations
   * @param a
   * @param b
   * @param epsilon
   * @return
   */
  bool compareRepresentations(T* a, T* b, const T& epsilon = std::numeric_limits<T>::epsilon())
  {
    bool close = false;
    for(int i = 0; i < 4; i++)
    {
      close = (epsilon > std::fabs(a[i] - b[i]));
      if(!close)
      {
        return close;
      }
    }
    return close;
  }

  /**
   * @brief printRepresentation
   * @param out
   * @param ro
   * @param label
   */
  void printRepresentation(std::ostream& out, T* ro, const std::string& label = std::string("Ro"))
  {
    out.precision(16);
    out << label << ro[0] << '\t' << ro[1] << '\t' << ro[2] << "\t" << ro[3] << std::endl;
  }

protected:
  RodriguesConverter()
  : OrientationConverter<DataArrayType, T>()
  {
  }

  explicit RodriguesConverter(DataArrayPointerType data)
  : OrientationConverter<DataArrayType, T>()
  {
    this->setInputData(data);
  }

public:
  RodriguesConverter(const RodriguesConverter&) = delete;            // Copy Constructor Not Implemented
  RodriguesConverter(RodriguesConverter&&) = delete;                 // Move Constructor Not Implemented
  RodriguesConverter& operator=(const RodriguesConverter&) = delete; // Copy Assignment Not Implemented
  RodriguesConverter& operator=(RodriguesConverter&&) = delete;      // Move Assignment Not Implemented
};

/* =============================================================================
 *
 * ===========================================================================*/

template <typename T>
class HomochoricSanityCheck
{
public:
  HomochoricSanityCheck(T* input, size_t stride)
  : m_Input(input)
  , m_Stride(stride)
  {
  }
  virtual ~HomochoricSanityCheck() = default;

  void sanityCheck(size_t start, size_t end) const
  {
    T* inPtr = m_Input + (start * m_Stride);

    for(size_t i = start; i < end; ++i)
    {
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    sanityCheck(r.begin(), r.end());
  }
#endif

private:
  T* m_Input = nullptr;
  size_t m_Stride = 0;
};

template <class DataArrayType, typename T>
class HomochoricConverter : public OrientationConverter<DataArrayType, T>
{
public:
  OC_CLASS_DEFINES(HomochoricConverter)

  ~HomochoricConverter() override = default;

  ebsdlib::orientations::Type getOrientationRepresentation()
  {
    return ebsdlib::orientations::Type::Homochoric;
  }

  void toEulers() override
  {
    OC_CONVERT_BODY_PREAMBLE(Homochoric, Euler);
  }

  void toOrientationMatrix() override
  {
    OC_CONVERT_BODY_PREAMBLE(Homochoric, OrientationMatrix);
  }

  void toQuaternion() override
  {
    OC_CONVERT_BODY_PREAMBLE(Homochoric, Quaternion);
  }

  void toAxisAngle() override
  {
    OC_CONVERT_BODY_PREAMBLE(Homochoric, AxisAngle);
  }

  void toRodrigues() override
  {
    OC_CONVERT_BODY_PREAMBLE(Homochoric, Rodrigues);
  }

  void toHomochoric() override
  {
    using PointerType = DataArrayPointerType;
    PointerType input = this->getInputData();
    PointerType output = std::dynamic_pointer_cast<DataArrayType>(input->deepCopy());
    this->setOutputData(output);
  }

  void toCubochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Homochoric, Cubochoric)
  }

  void toStereographic() override
  {
    OC_CONVERT_BODY_PREAMBLE(Homochoric, Stereographic)
  }

  void sanityCheckInputData() override
  {
    /* Apparently there is no sanity check for Homochoric, Odd. We place this
     * code here in case we come up with one, the parallel version is ready to
     * go
     */
#if 0
    DataArrayPointerType input = this->getInputData();
    T* inPtr = input->getPointer(0);
    size_t nTuples = input->getNumberOfTuples();
    int inStride = input->getNumberOfComponents();
#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), HomochoricSanityCheck<T>(inPtr, inStride), tbb::auto_partitioner());
  }
#else
    HomochoricSanityCheck<T> serial(inPtr, inStride);
    serial.sanityCheck(0, nTuples);
#endif
#endif
  }

  /**
   * @brief compareRepresentations
   * @param a
   * @param b
   * @param epsilon
   * @return
   */
  bool compareRepresentations(T* a, T* b, const T& epsilon = std::numeric_limits<T>::epsilon())
  {
    bool close = false;
    for(int i = 0; i < 3; i++)
    {
      close = (epsilon > std::fabs(a[i] - b[i]));
      if(!close)
      {
        return close;
      }
    }
    return close;
  }

  /**
   * @brief printRepresentation
   * @param out
   * @param ho
   * @param label
   */
  void printRepresentation(std::ostream& out, T* ho, const std::string& label = std::string("No"))
  {
    out.precision(16);
    out << label << ho[0] << '\t' << ho[1] << '\t' << ho[2] << std::endl;
  }

protected:
  HomochoricConverter()
  : OrientationConverter<DataArrayType, T>()
  {
  }

  explicit HomochoricConverter(DataArrayPointerType data)
  : OrientationConverter<DataArrayType, T>()
  {
    this->setInputData(data);
  }

public:
  HomochoricConverter(const HomochoricConverter&) = delete;            // Copy Constructor Not Implemented
  HomochoricConverter(HomochoricConverter&&) = delete;                 // Move Constructor Not Implemented
  HomochoricConverter& operator=(const HomochoricConverter&) = delete; // Copy Assignment Not Implemented
  HomochoricConverter& operator=(HomochoricConverter&&) = delete;      // Move Assignment Not Implemented
};

/* =============================================================================
 *
 * ===========================================================================*/

template <typename T>
class CubochoricSanityCheck
{
public:
  CubochoricSanityCheck(T* input, size_t stride)
  : m_Input(input)
  , m_Stride(stride)
  {
  }
  virtual ~CubochoricSanityCheck() = default;

  void sanityCheck(size_t start, size_t end) const
  {
    T* inPtr = m_Input + (start * m_Stride);

    for(size_t i = start; i < end; ++i)
    {
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    sanityCheck(r.begin(), r.end());
  }
#endif

private:
  T* m_Input = nullptr;
  size_t m_Stride = 0;
};

template <class DataArrayType, typename T>
class CubochoricConverter : public OrientationConverter<DataArrayType, T>
{
public:
  OC_CLASS_DEFINES(CubochoricConverter)

  ~CubochoricConverter() override = default;

  ebsdlib::orientations::Type getOrientationRepresentation()
  {
    return ebsdlib::orientations::Type::Cubochoric;
  }

  void toEulers() override
  {
    OC_CONVERT_BODY_PREAMBLE(Cubochoric, Euler);
  }

  void toOrientationMatrix() override
  {
    OC_CONVERT_BODY_PREAMBLE(Cubochoric, OrientationMatrix);
  }

  void toQuaternion() override
  {
    OC_CONVERT_BODY_PREAMBLE(Cubochoric, Quaternion);
  }

  void toAxisAngle() override
  {
    OC_CONVERT_BODY_PREAMBLE(Cubochoric, AxisAngle);
  }

  void toRodrigues() override
  {
    OC_CONVERT_BODY_PREAMBLE(Cubochoric, Rodrigues);
  }

  void toHomochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Cubochoric, Homochoric)
  }

  void toCubochoric() override
  {
    using PointerType = DataArrayPointerType;
    PointerType input = this->getInputData();
    PointerType output = std::dynamic_pointer_cast<DataArrayType>(input->deepCopy());
    this->setOutputData(output);
  }

  void toStereographic() override
  {
    OC_CONVERT_BODY_PREAMBLE(Cubochoric, Stereographic)
  }

  void sanityCheckInputData() override
  {
    /* Apparently there is no sanity check for Cubochoric, Odd. We place this
     * code here in case we come up with one, the parallel version is ready to
     * go
     */
#if 0
    DataArrayPointerType input = this->getInputData();
    T* inPtr = input->getPointer(0);
    size_t nTuples = input->getNumberOfTuples();
    int inStride = input->getNumberOfComponents();
#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), CubochoricSanityCheck<T>(inPtr, inStride), tbb::auto_partitioner());
#else
    CubochoricSanityCheck<T> serial(inPtr, inStride);
    serial.sanityCheck(0, nTuples);
#endif
#endif
  }

  /**
   * @brief compareRepresentations
   * @param a
   * @param b
   * @param epsilon
   * @return
   */
  bool compareRepresentations(T* a, T* b, const T& epsilon = std::numeric_limits<T>::epsilon())
  {
    bool close = false;
    for(int i = 0; i < 3; i++)
    {
      close = (epsilon > std::fabs(a[i] - b[i]));
      if(!close)
      {
        return close;
      }
    }
    return close;
  }

  /**
   * @brief printRepresentation
   * @param out
   * @param cu
   * @param label
   */
  void printRepresentation(std::ostream& out, T* cu, const std::string& label = std::string("Cu"))
  {
    out.precision(16);
    out << label << cu[0] << '\t' << cu[1] << '\t' << cu[2] << std::endl;
  }

protected:
  CubochoricConverter()
  : OrientationConverter<DataArrayType, T>()
  {
  }

  explicit CubochoricConverter(DataArrayPointerType data)
  : OrientationConverter<DataArrayType, T>()
  {
    this->setInputData(data);
  }

public:
  CubochoricConverter(const CubochoricConverter&) = delete;            // Copy Constructor Not Implemented
  CubochoricConverter(CubochoricConverter&&) = delete;                 // Move Constructor Not Implemented
  CubochoricConverter& operator=(const CubochoricConverter&) = delete; // Copy Assignment Not Implemented
  CubochoricConverter& operator=(CubochoricConverter&&) = delete;      // Move Assignment Not Implemented
};

/* =============================================================================
 *
 * ===========================================================================*/

template <typename T>
class StereographicSanityCheck
{
public:
  StereographicSanityCheck(T* input, size_t stride)
  : m_Input(input)
  , m_Stride(stride)
  {
  }
  virtual ~StereographicSanityCheck() = default;

  void sanityCheck(size_t start, size_t end) const
  {
    T* inPtr = m_Input + (start * m_Stride);

    for(size_t i = start; i < end; ++i)
    {
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    sanityCheck(r.begin(), r.end());
  }
#endif

private:
  T* m_Input = nullptr;
  size_t m_Stride = 0;
};

template <class DataArrayType, typename T>
class StereographicConverter : public OrientationConverter<DataArrayType, T>
{
public:
  OC_CLASS_DEFINES(StereographicConverter)

  ~StereographicConverter() override = default;

  ebsdlib::orientations::Type getOrientationRepresentation()
  {
    return ebsdlib::orientations::Type::Stereographic;
  }

  void toEulers() override
  {
    OC_CONVERT_BODY_PREAMBLE(Stereographic, Euler);
  }

  void toOrientationMatrix() override
  {
    OC_CONVERT_BODY_PREAMBLE(Stereographic, OrientationMatrix);
  }

  void toQuaternion() override
  {
    OC_CONVERT_BODY_PREAMBLE(Stereographic, Quaternion);
  }

  void toAxisAngle() override
  {
    OC_CONVERT_BODY_PREAMBLE(Stereographic, AxisAngle);
  }

  void toRodrigues() override
  {
    OC_CONVERT_BODY_PREAMBLE(Stereographic, Rodrigues);
  }

  void toHomochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Stereographic, Homochoric)
  }

  void toCubochoric() override
  {
    OC_CONVERT_BODY_PREAMBLE(Stereographic, Cubochoric)
  }

  void toStereographic() override
  {
    using PointerType = DataArrayPointerType;
    PointerType input = this->getInputData();
    PointerType output = std::dynamic_pointer_cast<DataArrayType>(input->deepCopy());
    this->setOutputData(output);
  }

  void sanityCheckInputData() override
  {
    /* Apparently there is no sanity check for Stereographic, Odd. We place this
     * code here in case we come up with one, the parallel version is ready to
     * go
     */
#if 0
    DataArrayPointerType input = this->getInputData();
    T* inPtr = input->getPointer(0);
    size_t nTuples = input->getNumberOfTuples();
    int inStride = input->getNumberOfComponents();
#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nTuples), StereographicSanityCheck<T>(inPtr, inStride), tbb::auto_partitioner());
#else
    StereographicSanityCheck<T> serial(inPtr, inStride);
    serial.sanityCheck(0, nTuples);
#endif
#endif
  }

  /**
   * @brief compareRepresentations
   * @param a
   * @param b
   * @param epsilon
   * @return
   */
  bool compareRepresentations(T* a, T* b, const T& epsilon = std::numeric_limits<T>::epsilon())
  {
    bool close = false;
    for(int i = 0; i < 3; i++)
    {
      close = (epsilon > std::fabs(a[i] - b[i]));
      if(!close)
      {
        return close;
      }
    }
    return close;
  }

  /**
   * @brief printRepresentation
   * @param out
   * @param sp
   * @param label
   */
  void printRepresentation(std::ostream& out, T* sp, const std::string& label = std::string("St"))
  {
    out.precision(16);
    out << label << sp[0] << '\t' << sp[1] << '\t' << sp[2] << std::endl;
  }

protected:
  StereographicConverter()
  : OrientationConverter<DataArrayType, T>()
  {
  }

  explicit StereographicConverter(DataArrayPointerType data)
  : OrientationConverter<DataArrayType, T>()
  {
    this->setInputData(data);
  }

public:
  StereographicConverter(const StereographicConverter&) = delete;            // Copy Constructor Not Implemented
  StereographicConverter(StereographicConverter&&) = delete;                 // Move Constructor Not Implemented
  StereographicConverter& operator=(const StereographicConverter&) = delete; // Copy Assignment Not Implemented
  StereographicConverter& operator=(StereographicConverter&&) = delete;      // Move Assignment Not Implemented
};
} // namespace ebsdlib
