/* ============================================================================
 * Copyright (c) 2023-2023 BlueQuartz Software, LLC
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#pragma once

#include "EbsdLib/Core/EbsdSetGetMacros.h"
#include "EbsdLib/EbsdLib.h"

#include "CtfConstants.h"
#include "CtfPhase.h"
#include "CtfReader.h"

#include <hdf5.h>

#include <list>
#include <set>
#include <string>
#include <vector>

#define H5OINA_HEADER_PROP(type, name)                                                                                                                                                                 \
private:                                                                                                                                                                                               \
  type m_##name = {};                                                                                                                                                                                  \
                                                                                                                                                                                                       \
public:                                                                                                                                                                                                \
  void set##name(type value)                                                                                                                                                                           \
  {                                                                                                                                                                                                    \
    m_##name = value;                                                                                                                                                                                  \
  }                                                                                                                                                                                                    \
  type get##name()                                                                                                                                                                                     \
  {                                                                                                                                                                                                    \
    return m_##name;                                                                                                                                                                                   \
  }

#define H5OINA_DATA_PTR_PROP(name, var, type)                                                                                                                                                          \
private:                                                                                                                                                                                               \
  std::vector<type> m_##name = {};                                                                                                                                                                     \
                                                                                                                                                                                                       \
public:                                                                                                                                                                                                \
  type* get##name##Pointer()                                                                                                                                                                           \
  {                                                                                                                                                                                                    \
    return m_##name.data();                                                                                                                                                                            \
  }

class EbsdLib_EXPORT H5OINAReader : public EbsdReader
{

public:
  using Self = H5OINAReader;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer();

  EBSD_STATIC_NEW_MACRO(H5OINAReader)

  H5OINAReader();

  H5OINAReader(const H5OINAReader&) = delete;            // Copy Constructor Not Implemented
  H5OINAReader(H5OINAReader&&) = delete;                 // Move Constructor Not Implemented
  H5OINAReader& operator=(const H5OINAReader&) = delete; // Copy Assignment Not Implemented
  H5OINAReader& operator=(H5OINAReader&&) = delete;      // Move Assignment Not Implemented

  H5OINA_HEADER_PROP(std::string, AcquisitionData)
  H5OINA_HEADER_PROP(float, AcquisitionSpeed)
  H5OINA_HEADER_PROP(float, AcquisitionTime)
  H5OINA_HEADER_PROP(float, BeamVoltage)
  H5OINA_HEADER_PROP(std::vector<float>, DetectorOrientationEuler)
  H5OINA_HEADER_PROP(float, Magnification)
  H5OINA_HEADER_PROP(std::string, ProjectFile)
  H5OINA_HEADER_PROP(std::string, ProjectLabel)
  H5OINA_HEADER_PROP(std::string, ProjectNotes)
  H5OINA_HEADER_PROP(float, ScanningRotationAngle)
  H5OINA_HEADER_PROP(std::string, SiteNotes)
  H5OINA_HEADER_PROP(std::vector<float>, SpecimenOrientationEuler)
  H5OINA_HEADER_PROP(float, TiltAxis)
  H5OINA_HEADER_PROP(float, TiltAngle)
  H5OINA_HEADER_PROP(int32_t, XCells)
  H5OINA_HEADER_PROP(float, XStep)
  H5OINA_HEADER_PROP(int32_t, YCells)
  H5OINA_HEADER_PROP(float, YStep)

  H5OINA_DATA_PTR_PROP(BandContrast, BandContrast, uint8_t)
  H5OINA_DATA_PTR_PROP(BandSlope, BandSlope, uint8_t)
  H5OINA_DATA_PTR_PROP(Bands, Bands, uint8_t)
  H5OINA_DATA_PTR_PROP(Error, Error, uint8_t)
  H5OINA_DATA_PTR_PROP(Euler, Euler, float)
  H5OINA_DATA_PTR_PROP(MeanAngularDeviation, MeanAngularDeviation, float)
  H5OINA_DATA_PTR_PROP(Phase, Phase, uint8_t)
  H5OINA_DATA_PTR_PROP(X, X, float)
  H5OINA_DATA_PTR_PROP(Y, Y, float)

  EBSD_INSTANCE_PROPERTY(std::vector<CtfPhase::Pointer>, PhaseVector)

  void setReadPatternData(bool value);
  uint16_t* getPatternData();
  void getPatternDims(std::array<int32_t,2 > dims);

  /**
   * @brief Returns the pointer to the data for a given feature
   * @param featureName The name of the feature to return the pointer to.
   */
  void* getPointerByName(const std::string& featureName) override;

  /**
   * @brief Returns an enumeration value that depicts the numerical
   * primitive type that the data is stored as (Int, Float, etc).
   * @param featureName The name of the feature.
   */
  EbsdLib::NumericTypes::Type getPointerType(const std::string& featureName) override;

  /**
   * @brief Returns the name of the class for H5OINAReader
   */
  std::string getNameOfClass() const;

  /**
   * @brief Returns the name of the class for H5OINAReader
   */
  static std::string ClassName();

  ~H5OINAReader() override;

  /**
   * @brief The HDF5 path to find the EBSD data
   */
  void setHDF5Path(const std::string& value);

  /**
   * @brief Getter property for HDF5Path
   * @return Value of HDF5Path
   */
  std::string getHDF5Path() const;

  std::string getOINAVersion() const;

  /**
   * @brief Returns the X Dimension of the data. This method is pure virtual
   * and should be implemented by subclasses.
   */
  int getXDimension() override;
  /**
   * @brief Sets the X Dimension of the data. This method is pure virtual
   * and should be implemented by subclasses.
   */
  void setXDimension(int xdim) override;
  /**
   * @brief Returns the Y Dimension of the data. This method is pure virtual
   * and should be implemented by subclasses.
   */
  int getYDimension() override;
  /**
   * @brief Sets the Y Dimension of the data. This method is pure virtual
   * and should be implemented by subclasses.
   */
  void setYDimension(int ydim) override;

  /**
   * @brief Reads the file
   * @return error condition
   */
  int readFile() override;

  /**
   * @brief readScanNames
   * @return
   */
  int readScanNames(std::list<std::string>& names);

  /**
   * @brief Reads the header section of the file
   * @param Valid HDF5 Group ID
   * @return error condition
   */
  int readHeader(hid_t parId);

  /**
   * @brief Reads ONLY the header portion of the TSL .ang file
   * @return 1 on success
   */
  int readHeaderOnly() override;

  /**
   * @brief Returns a vector of AngPhase objects corresponding to the phases
   * present in the file
   */
  // std::vector<AngPhase::Pointer> getPhases() { return m_Phases; }

  /**
   * @brief Sets the names of the arrays to read out of the file
   * @param names
   */
  void setArraysToRead(const std::set<std::string>& names);

  /**
   * @brief Over rides the setArraysToReads to tell the reader to load ALL the data from the HDF5 file. If the
   * ArrayNames to read is empty and this is true then all arrays will be read.
   * @param b
   */
  void readAllArrays(bool b);

protected:
  /**
   * @brief Reads the data associated with HKL Families for a given phase.
   * @param hklGid Valid HDF5 Group ID where the HKL Family data is located.
   * @param phase The AngPhase to parse the HKL Family data
   */
  int readHKLFamilies(hid_t hklGid, const CtfPhase::Pointer& phase);

  /**
   * @brief Reads the data section of the file
   * @param Valid HDF5 Group ID
   * @return error condition
   */
  int readData(hid_t parId);

private:
  std::string m_HDF5Path = {};

  bool m_ReadPatternData = false;
  std::string m_OINAVersion = {};

  std::set<std::string> m_ArrayNames;
  bool m_ReadAllArrays = true;
};
