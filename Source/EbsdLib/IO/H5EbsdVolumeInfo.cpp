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

#include "H5EbsdVolumeInfo.h"

#include <iostream>
#include <string>

#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/HKL/CtfConstants.h"
#include "EbsdLib/IO/TSL/AngConstants.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

#define EBSD_VOLREADER_READ_HEADER(fileId, path, var)                                                                                                                                                  \
  err = H5Lite::readScalarDataset(fileId, path, var);                                                                                                                                                  \
  if(err < 0)                                                                                                                                                                                          \
  {                                                                                                                                                                                                    \
    std::cout << "H5EbsdVolumeInfo Error: Could not load header value for " << path;                                                                                                                   \
    err = H5Utilities::closeFile(fileId);                                                                                                                                                              \
    return err;                                                                                                                                                                                        \
  }

#define EBSD_VOLREADER_READ_VECTOR3_HEADER(fileId, path, var, type)                                                                                                                                    \
  {                                                                                                                                                                                                    \
    err = H5Lite::readPointerDataset(fileId, path, var.data());                                                                                                                                        \
    if(err < 0)                                                                                                                                                                                        \
    {                                                                                                                                                                                                  \
      std::cout << "H5EbsdVolumeInfo Error: Could not load header (as vector) for " << path;                                                                                                           \
      err = H5Utilities::closeFile(fileId);                                                                                                                                                            \
      return err;                                                                                                                                                                                      \
    }                                                                                                                                                                                                  \
  }

#define EBSD_VOLREADER_READ_HEADER_CAST(fileId, path, var, m_msgType, cast)                                                                                                                            \
  {                                                                                                                                                                                                    \
    cast t;                                                                                                                                                                                            \
    err = H5Lite::readScalarDataset(fileId, path, t);                                                                                                                                                  \
    if(err < 0)                                                                                                                                                                                        \
    {                                                                                                                                                                                                  \
      std::cout << "H5EbsdVolumeInfo Error: Could not load header value (with cast) for " << path;                                                                                                     \
      err = H5Utilities::closeFile(fileId);                                                                                                                                                            \
      return err;                                                                                                                                                                                      \
    }                                                                                                                                                                                                  \
    var = static_cast<m_msgType>(t);                                                                                                                                                                   \
  }

using namespace H5Support;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
H5EbsdVolumeInfo::H5EbsdVolumeInfo()
: m_ErrorCode(0)
, m_ErrorMessage("")
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
H5EbsdVolumeInfo::~H5EbsdVolumeInfo() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5EbsdVolumeInfo::invalidateCache()
{
  m_ValuesAreCached = (false);
  m_XDim = (0);
  m_YDim = (0);
  m_ZDim = (0);
  m_XRes = (0.0f);
  m_YRes = (0.0f);
  m_ZRes = (0.0f);
  m_ZStart = (0);
  m_ZEnd = (0);
  m_NumPhases = (0);
  m_Manufacturer = "";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EbsdVolumeInfo::updateToLatestVersion()
{
  invalidateCache();
  // Open the file with Read/Write access
  hid_t fileId = H5Utilities::openFile(m_FileName, false);
  if(fileId < 0)
  {
    // std::cout << "Error Opening file '" << m_FileName << "'" << std::endl;
    return -1;
  }
  // This sentinel will make sure the file is closed and the errors turned back on when we
  // exit the function
  H5ScopedFileSentinel sentinel(fileId, true);

  // Update any existing datasets/attributes with new datasets/attributes/

  return 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EbsdVolumeInfo::readVolumeInfo()
{
  int err = -1;
  m_ValuesAreCached = false;
  int retErr = 0;
  hid_t fileId = H5Utilities::openFile(m_FileName, true);
  if(fileId < 0)
  {
    // std::cout << "Error Opening file '" << m_FileName << "'" << std::endl;
    return -1;
  }
  H5ScopedFileSentinel sentinel(fileId, true);

  m_FileVersion = 0;
  // Attempt to read the file version number. If it is not there that is OK as early h5ebsd
  // files did not have this information written.
  err = H5Lite::readScalarAttribute(fileId, "/", EbsdLib::H5Ebsd::FileVersionStr, m_FileVersion);

  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::ZStartIndex, m_ZStart);
  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::ZEndIndex, m_ZEnd);
  m_ZDim = m_ZEnd - m_ZStart + 1; // The range is inclusive (zStart, zEnd)
  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::XPoints, m_XDim);
  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::YPoints, m_YDim);
  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::XResolution, m_XRes);
  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::YResolution, m_YRes);
  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::ZResolution, m_ZRes);

  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::StackingOrder, m_StackingOrder);
  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::SampleTransformationAngle, m_SampleTransformationAngle);
  EBSD_VOLREADER_READ_VECTOR3_HEADER(fileId, EbsdLib::H5Ebsd::SampleTransformationAxis, m_SampleTransformationAxis, float);
  EBSD_VOLREADER_READ_HEADER(fileId, EbsdLib::H5Ebsd::EulerTransformationAngle, m_EulerTransformationAngle);
  EBSD_VOLREADER_READ_VECTOR3_HEADER(fileId, EbsdLib::H5Ebsd::EulerTransformationAxis, m_EulerTransformationAxis, float);

  // Read the manufacturer from the file
  m_Manufacturer = "";
  std::string data;
  err = H5Lite::readStringDataset(fileId, EbsdLib::H5Ebsd::Manufacturer, data);
  if(err < 0)
  {
    std::cout << "H5EbsdVolumeInfo Error: Could not load header value for " << EbsdLib::H5Ebsd::Manufacturer << std::endl;
    err = H5Utilities::closeFile(fileId);
    return err;
  }
  m_Manufacturer = data;

  // Get the Number of Phases in the Material
  // DO NOT Use the accessor methods below to get variables. Directly access them otherwise you
  // will cause an infinite recursion to occur.
  std::string index = EbsdStringUtils::number<int32_t>(m_ZStart);
  hid_t gid = H5Gopen(fileId, index.c_str(), H5P_DEFAULT);
  if(gid > 0)
  {
    hid_t headerId = H5Gopen(gid, EbsdLib::H5Ebsd::Header.c_str(), H5P_DEFAULT);
    if(headerId > 0)
    {
      hid_t phasesGid = H5Gopen(headerId, EbsdLib::H5Ebsd::Phases.c_str(), H5P_DEFAULT);
      if(phasesGid > 0)
      {
        std::list<std::string> names;
        err = H5Utilities::getGroupObjects(phasesGid, H5Utilities::CustomHDFDataTypes::Group, names);
        if(err >= 0)
        {
          m_NumPhases = static_cast<int>(names.size());
        }
        H5Gclose(phasesGid);
      }
      H5Gclose(headerId);
    }

    // Now read out the names of the data arrays in the file

    hid_t dataGid = H5Gopen(gid, EbsdLib::H5Ebsd::Data.c_str(), H5P_DEFAULT);
    if(dataGid > 0)
    {
      std::list<std::string> names;
      err = H5Utilities::getGroupObjects(dataGid, H5Utilities::CustomHDFDataTypes::Dataset, names);
      if(err >= 0)
      {
        for(auto& name : names)
        {
          m_DataArrayNames.insert(name);
        }
      }
      H5Gclose(dataGid);
    }
    H5Gclose(gid);
  }

  // we are going to selectively replace some of the data array names with some common names instead
  if(m_Manufacturer == EbsdLib::Ang::Manufacturer)
  {
    if(m_DataArrayNames.count(EbsdLib::Ang::Phi1) != 0 && m_DataArrayNames.count(EbsdLib::Ang::Phi) != 0 && m_DataArrayNames.count(EbsdLib::Ang::Phi2) != 0)
    {
      m_DataArrayNames.erase(EbsdLib::Ang::Phi1);
      m_DataArrayNames.erase(EbsdLib::Ang::Phi);
      m_DataArrayNames.erase(EbsdLib::Ang::Phi2);
      m_DataArrayNames.insert(EbsdLib::CellData::EulerAngles);
    }
    if(m_DataArrayNames.count(EbsdLib::Ang::PhaseData) != 0)
    {
      m_DataArrayNames.erase(EbsdLib::Ang::PhaseData);
      m_DataArrayNames.insert(EbsdLib::CellData::Phases);
    }
  }
  else if(m_Manufacturer == EbsdLib::Ctf::Manufacturer)
  {
    if(m_DataArrayNames.count(EbsdLib::Ctf::Euler1) != 0 && m_DataArrayNames.count(EbsdLib::Ctf::Euler2) != 0 && m_DataArrayNames.count(EbsdLib::Ctf::Euler3) != 0)
    {
      m_DataArrayNames.erase(EbsdLib::Ctf::Euler1);
      m_DataArrayNames.erase(EbsdLib::Ctf::Euler2);
      m_DataArrayNames.erase(EbsdLib::Ctf::Euler3);
      m_DataArrayNames.insert(EbsdLib::CellData::EulerAngles);
    }
    if(m_DataArrayNames.count(EbsdLib::Ctf::Phase) != 0)
    {
      m_DataArrayNames.erase(EbsdLib::Ctf::Phase);
      m_DataArrayNames.insert(EbsdLib::CellData::Phases);
    }
  }

  m_ValuesAreCached = true;
  return retErr;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
uint32_t H5EbsdVolumeInfo::getFileVersion()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return 0;
    }
  }
  return m_FileVersion;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EbsdVolumeInfo::getDimsAndResolution(int64_t& xDim, int64_t& yDim, int64_t& zDim, float& xRes, float& yRes, float& zRes)
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return -1;
    }
  }
  if(m_ValuesAreCached)
  {
    xDim = m_XDim;
    yDim = m_YDim;
    zDim = m_ZDim;
    xRes = m_XRes;
    yRes = m_YRes;
    zRes = m_ZRes;
    err = 0;
  }
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EbsdVolumeInfo::getDims(int64_t& xDim, int64_t& yDim, int64_t& zDim)
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return -1;
    }
  }
  if(m_ValuesAreCached)
  {
    xDim = m_XDim;
    yDim = m_YDim;
    zDim = m_ZDim;
    err = 0;
  }
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EbsdVolumeInfo::getSpacing(float& xRes, float& yRes, float& zRes)
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return -1;
    }
  }
  if(m_ValuesAreCached)
  {
    xRes = m_XRes;
    yRes = m_YRes;
    zRes = m_ZRes;
    err = 0;
  }
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string H5EbsdVolumeInfo::getManufacturer()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return std::string("");
    }
  }
  return m_Manufacturer;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EbsdVolumeInfo::getNumSlices()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return 0;
    }
  }
  return m_ZDim;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EbsdVolumeInfo::getZStart()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return 0;
    }
  }
  return m_ZStart;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EbsdVolumeInfo::getZEnd()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return 0;
    }
  }
  return m_ZEnd;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EbsdVolumeInfo::getNumPhases()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return 0;
    }
  }
  return m_NumPhases;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
uint32_t H5EbsdVolumeInfo::getStackingOrder()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return EbsdLib::RefFrameZDir::UnknownRefFrameZDirection;
    }
  }
  return m_StackingOrder;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
float H5EbsdVolumeInfo::getSampleTransformationAngle()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return 0.0;
    }
  }
  return m_SampleTransformationAngle;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::array<float, 3> H5EbsdVolumeInfo::getSampleTransformationAxis()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      std::array<float, 3> axis = {0.0F, 0.0F, 1.0F};
      return axis;
    }
  }
  return m_SampleTransformationAxis;
}
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
float H5EbsdVolumeInfo::getEulerTransformationAngle()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return 0.0;
    }
  }
  return m_EulerTransformationAngle;
}
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::array<float, 3> H5EbsdVolumeInfo::getEulerTransformationAxis()
{
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      std::array<float, 3> axis = {0.0F, 0.0F, 1.0F};
      return axis;
    }
  }
  return m_EulerTransformationAxis;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::set<std::string> H5EbsdVolumeInfo::getDataArrayNames()
{
  std::set<std::string> empty;
  int err = -1;
  if(!m_ValuesAreCached)
  {
    err = readVolumeInfo();
    if(err < 0)
    {
      return empty;
    }
  }
  return m_DataArrayNames;
}

// -----------------------------------------------------------------------------
H5EbsdVolumeInfo::Pointer H5EbsdVolumeInfo::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
void H5EbsdVolumeInfo::setErrorMessage(const std::string& value)
{
  m_ErrorMessage = value;
}

// -----------------------------------------------------------------------------
std::string H5EbsdVolumeInfo::getErrorMessage() const
{
  return m_ErrorMessage;
}

// -----------------------------------------------------------------------------
void H5EbsdVolumeInfo::setFileName(const std::string& value)
{
  m_FileName = value;
}

// -----------------------------------------------------------------------------
std::string H5EbsdVolumeInfo::getFileName() const
{
  return m_FileName;
}

// -----------------------------------------------------------------------------
std::string H5EbsdVolumeInfo::getNameOfClass() const
{
  return std::string("H5EbsdVolumeInfo");
}

// -----------------------------------------------------------------------------
std::string H5EbsdVolumeInfo::ClassName()
{
  return std::string("H5EbsdVolumeInfo");
}
