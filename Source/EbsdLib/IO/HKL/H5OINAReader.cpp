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

#include "H5OINAReader.h"

#include "CtfHeaderEntry.h"
#include "CtfReader.h"

#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

#include <cstdint>
#include <iostream>
#include <vector>

using namespace H5Support;

// -----------------------------------------------------------------------------
H5OINAReader::H5OINAReader()
{
  m_HeaderMap.clear();
  // Initialize the map of header key to header value
  m_HeaderMap[EbsdLib::H5OINA::AcquisitionData] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::H5OINA::AcquisitionData);
  m_HeaderMap[EbsdLib::H5OINA::AcquisitionSpeed] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::H5OINA::AcquisitionSpeed);
  m_HeaderMap[EbsdLib::H5OINA::AcquisitionTime] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::H5OINA::AcquisitionTime);
  m_HeaderMap[EbsdLib::H5OINA::BeamVoltage] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::H5OINA::BeamVoltage);
  m_HeaderMap[EbsdLib::H5OINA::DetectorOrientationEuler] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::H5OINA::DetectorOrientationEuler);
  m_HeaderMap[EbsdLib::H5OINA::Magnification] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::H5OINA::Magnification);
  m_HeaderMap[EbsdLib::H5OINA::ProjectFile] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::H5OINA::ProjectFile);
  m_HeaderMap[EbsdLib::H5OINA::ProjectLabel] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::H5OINA::ProjectLabel);
  m_HeaderMap[EbsdLib::H5OINA::ProjectNotes] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::H5OINA::ProjectNotes);
  m_HeaderMap[EbsdLib::H5OINA::ScanningRotationAngle] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::H5OINA::ScanningRotationAngle);
  m_HeaderMap[EbsdLib::H5OINA::SiteNotes] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::H5OINA::SiteNotes);
  m_HeaderMap[EbsdLib::H5OINA::SpecimenOrientationEuler] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::H5OINA::SpecimenOrientationEuler);
  m_HeaderMap[EbsdLib::H5OINA::TiltAxis] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::H5OINA::TiltAxis);
  m_HeaderMap[EbsdLib::H5OINA::TiltAngle] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::H5OINA::TiltAngle);

  m_HeaderMap[EbsdLib::Ctf::XCells] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::XCells);
  m_HeaderMap[EbsdLib::Ctf::XStep] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::XStep);
  m_HeaderMap[EbsdLib::Ctf::YCells] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::YCells);
  m_HeaderMap[EbsdLib::Ctf::YStep] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::YStep);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
H5OINAReader::~H5OINAReader()
{
  // this->deallocateArrayData<uint8_t>(m_PatternData);
}

int H5OINAReader::getXDimension()
{
  return m_XCells;
}
/**
 * @brief Sets the X Dimension of the data. This method is pure virtual
 * and should be implemented by subclasses.
 */
void H5OINAReader::setXDimension(int xdim)
{
  m_XCells = xdim;
}
/**
 * @brief Returns the Y Dimension of the data. This method is pure virtual
 * and should be implemented by subclasses.
 */
int H5OINAReader::getYDimension()
{
  return m_YCells;
}
/**
 * @brief Sets the Y Dimension of the data. This method is pure virtual
 * and should be implemented by subclasses.
 */
void H5OINAReader::setYDimension(int ydim)
{
  m_YCells = ydim;
}

void* H5OINAReader::getPointerByName(const std::string& featureName)
{
  void* ptr = nullptr;
  if(featureName == EbsdLib::H5OINA::BandContrast)
  {
    return m_BandContrast.data();
  }
  if(featureName == EbsdLib::H5OINA::BandSlope)
  {
    return m_BandSlope.data();
  }
  if(featureName == EbsdLib::H5OINA::Bands)
  {
    return m_Bands.data();
  }
  if(featureName == EbsdLib::H5OINA::Error)
  {
    return m_Error.data();
  }
  if(featureName == EbsdLib::H5OINA::Euler)
  {
    return m_Euler.data();
  }
  if(featureName == EbsdLib::H5OINA::MeanAngularDeviation)
  {
    return m_MeanAngularDeviation.data();
  }
  if(featureName == EbsdLib::H5OINA::Phase)
  {
    return m_Phase.data();
  }
  if(featureName == EbsdLib::H5OINA::X)
  {
    return m_X.data();
  }
  if(featureName == EbsdLib::H5OINA::Y)
  {
    return m_Y.data();
  }

  return nullptr;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::NumericTypes::Type H5OINAReader::getPointerType(const std::string& featureName)
{
  // std::cout << "featureName: " << featureName << std::endl;
  if(featureName == EbsdLib::H5OINA::BandContrast)
  {
    return EbsdLib::NumericTypes::Type::UInt8;
  }
  if(featureName == EbsdLib::H5OINA::BandSlope)
  {
    return EbsdLib::NumericTypes::Type::UInt8;
  }
  if(featureName == EbsdLib::H5OINA::Bands)
  {
    return EbsdLib::NumericTypes::Type::UInt8;
  }
  if(featureName == EbsdLib::H5OINA::Error)
  {
    return EbsdLib::NumericTypes::Type::UInt8;
  }
  if(featureName == EbsdLib::H5OINA::Euler)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::H5OINA::MeanAngularDeviation)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::H5OINA::Phase)
  {
    return EbsdLib::NumericTypes::Type::UInt8;
  }
  if(featureName == EbsdLib::H5OINA::X)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::H5OINA::Y)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }

  // std::cout << "THIS IS NOT GOOD. Featurename: " << featureName << " was not found in the list" << std::endl;
  return EbsdLib::NumericTypes::Type::UnknownNumType;
}

void H5OINAReader::setReadPatternData(bool value)
{
  m_ReadPatternData = value;
}

uint16_t* H5OINAReader::getPatternData()
{
  return nullptr;
}
void H5OINAReader::getPatternDims(std::array<int32_t, 2> dims)
{
  ;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OINAReader::readFile()
{
  int err = -1;
  if(m_HDF5Path.empty())
  {
    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error: HDF5 Path is empty.";
    setErrorCode(-1);
    setErrorMessage(str);
    return err;
  }

  hid_t fileId = H5Utilities::openFile(getFileName(), true);
  if(fileId < 0)
  {
    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error: Could not open HDF5 file '" << getFileName() << "'";
    setErrorCode(-2);
    setErrorMessage(str);
    return err;
  }

  err = H5Support::H5Lite::readStringDataset(fileId, EbsdLib::H5OINA::FormatVersion, m_OINAVersion);
  if(err < 0)
  {
  }

  H5ScopedFileSentinel sentinel(fileId, false);
  hid_t gid = H5Gopen(fileId, m_HDF5Path.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error: Could not open path '" << m_HDF5Path << "'";
    setErrorCode(-90020);
    setErrorMessage(str);
    return getErrorCode();
  }
  sentinel.addGroupId(gid);

  hid_t ebsdGid = H5Gopen(gid, EbsdLib::H5OINA::EBSD.c_str(), H5P_DEFAULT);
  if(ebsdGid < 0)
  {
    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error: Could not open 'EBSD' Group";
    setErrorCode(-90007);
    setErrorMessage(str);
    return getErrorCode();
  }
  sentinel.addGroupId(ebsdGid);

  // Read all the header information
  err = readHeader(ebsdGid);
  if(err < 0)
  {
    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error: could not read header";
    setErrorCode(-900021);
    setErrorMessage(str);
    return getErrorCode();
  }

  // Read data
  err = readData(ebsdGid);
  if(err < 0)
  {
    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error: could not read data. Internal Error code " << err << " generated.";
    setErrorCode(-900022);
    setErrorMessage(str);
    return getErrorCode();
  }

  //  std::vector<int64_t> indexMap;
  //  std::pair<int, std::string> result = fixOrderOfData(indexMap);
  //
  //  if(result.first < 0)
  //  {
  //    std::cout << result.second << std::endl;
  //    return result.first;
  //  }
  //
  //  std::vector<std::string> arrayNames = {"Phi1", "Phi", "Phi2", "X Position", "Y Position", "Image Quality", "Confidence Index", "PhaseData", "SEM Signal", "Fit"};
  //  for(const auto& arrayName : arrayNames)
  //  {
  //    void* oldArray = getPointerByName(arrayName);
  //
  //    if(getPointerType(arrayName) == EbsdLib::NumericTypes::Type::Float)
  //    {
  //      CopyTupleUsingIndexList<float>(oldArray, indexMap);
  //    }
  //    else if(getPointerType(arrayName) == EbsdLib::NumericTypes::Type::Int32)
  //    {
  //      CopyTupleUsingIndexList<int32_t>(oldArray, indexMap);
  //    }
  //    else
  //    {
  //      std::cout << "Type returned was not of Float or int32. The Array name probably isn't correct." << std::endl;
  //    }
  //  }

  return getErrorCode();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OINAReader::readHeaderOnly()
{
  int err = -1;

  hid_t fileId = H5Utilities::openFile(getFileName(), true);
  if(fileId < 0)
  {
    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error: Could not open HDF5 file '" << getFileName() << "'";
    setErrorCode(-10);
    setErrorMessage(str);
    return getErrorCode();
  }
  H5ScopedFileSentinel sentinel(fileId, false);

  if(m_HDF5Path.empty())
  {
    std::list<std::string> names;
    err = H5Utilities::getGroupObjects(fileId, H5Utilities::CustomHDFDataTypes::Group, names);

    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error (Internal HDF5 Path is empty): The name of the scan was not specified. There are " << names.size() << " scans available. ";
    int nameCount = static_cast<int>(names.size());
    if(nameCount < 10)
    {
      ss << " The scan names are: ";
    }
    else
    {
      nameCount = 10;
      ss << " The first 10 scan names are: ";
    }
    for(const auto& name : names)
    {
      ss << name << "\n";
    }
    setErrorCode(-11);
    setErrorMessage(str);
    return getErrorCode();
  }

  // Read and parse the OINA Format Version.
  if(H5Lite::datasetExists(fileId, EbsdLib::H5OINA::FormatVersion))
  {
    err = H5Lite::readStringDataset(fileId, EbsdLib::H5OINA::FormatVersion, m_OINAVersion);
  }

  hid_t gid = H5Gopen(fileId, m_HDF5Path.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error: Could not open path '" << m_HDF5Path << "'";
    setErrorCode(-12);
    setErrorMessage(str);
    return getErrorCode();
  }
  sentinel.addGroupId(gid);

  hid_t ebsdGid = H5Gopen(gid, EbsdLib::H5OINA::EBSD.c_str(), H5P_DEFAULT);
  if(ebsdGid < 0)
  {
    setErrorMessage("H5OINAReader Error: Could not open 'EBSD' Group");
    setErrorCode(-90007);
    return getErrorCode();
  }
  sentinel.addGroupId(ebsdGid);

  err = readHeader(ebsdGid);

  return getErrorCode();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OINAReader::readScanNames(std::list<std::string>& names)
{
  int err = -1;
  hid_t fileId = H5Utilities::openFile(getFileName(), true);
  if(fileId < 0)
  {
    std::string str;
    std::stringstream ss(str);
    ss << getNameOfClass() << "Error: Could not open HDF5 file '" << getFileName() << "'";
    setErrorCode(-20);
    setErrorMessage(str);
    names.clear();
    return getErrorCode();
  }
  H5ScopedFileSentinel sentinel(fileId, false);

  err = H5Utilities::getGroupObjects(fileId, H5Utilities::CustomHDFDataTypes::Group, names);
  setErrorCode(err);
  return err;
}

template <typename T>
int32_t ReadH5OINAHeaderScalarValue(H5OINAReader* c, const std::string& key, hid_t gid, T& value)
{
  T t;
  herr_t err = H5Lite::readScalarDataset(gid, key, value);
  if(err < 0)
  {
    std::stringstream ss;
    ss << c->getNameOfClass() << ": The header value for '" << key << "' was not found in the H5OINA file.";
    c->setErrorCode(-9056);
    c->setErrorMessage(ss.str());
    return err;
  }
  return 0;
}

template <typename T>
int32_t ReadH5OINAHeaderVectorValue(H5OINAReader* c, const std::string& key, hid_t gid, std::vector<T>& value)
{
  T t;
  herr_t err = H5Lite::readVectorDataset(gid, key, value);
  if(err < 0)
  {
    std::stringstream ss;
    ss << c->getNameOfClass() << ": The header value for '" << key << "' was not found in the H5OINA file.";
    c->setErrorCode(-9056);
    c->setErrorMessage(ss.str());
    return err;
  }
  return 0;
}

int32_t ReadH5OINAHeaderStringValue(H5OINAReader* c, const std::string& key, hid_t gid, std::string& value)
{
  herr_t err = H5Lite::readStringDataset(gid, key, value);
  if(err < 0)
  {
    std::stringstream ss;
    ss << c->getNameOfClass() << ": The header value for '" << key << "' was not found in the H5OINA file.";
    c->setErrorCode(-90002);
    c->setErrorMessage(ss.str());
    return err;
  }
  return 0;
}

// -----------------------------------------------------------------------------
int H5OINAReader::readHeader(hid_t parId)
{
  using CtfHeaderFloatType = CtfHeaderEntry<float, FloatHeaderParser>;
  using CtfHeaderIntType = CtfHeaderEntry<int, Int32HeaderParser>;
  int err = -1;

  hid_t gid = H5Gopen(parId, EbsdLib::H5OINA::Header.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    setErrorCode(-90008);
    setErrorMessage("H5OINAReader Error: Could not open 'Header' Group");
    return -1;
  }
  H5ScopedGroupSentinel sentinel(gid, false);

  ReadH5OINAHeaderScalarValue(this, EbsdLib::H5OINA::XCells, gid, m_XCells);
  ReadH5OINAHeaderScalarValue(this, EbsdLib::H5OINA::XStep, gid, m_XStep);
  ReadH5OINAHeaderScalarValue(this, EbsdLib::H5OINA::YCells, gid, m_YCells);
  ReadH5OINAHeaderScalarValue(this, EbsdLib::H5OINA::YStep, gid, m_YStep);

  setNumberOfElements(getXCells() * getYCells());

  // Figure out the Format version
  if(m_OINAVersion.find(EbsdLib::H5OINA::FormatVersion_2) != std::string::npos)
  {
  }
  if(m_OINAVersion.find(EbsdLib::H5OINA::FormatVersion_3) != std::string::npos)
  {
  }
  if(m_OINAVersion.find(EbsdLib::H5OINA::FormatVersion_4) != std::string::npos)
  {
  }
  if(m_OINAVersion.find(EbsdLib::H5OINA::FormatVersion_5) != std::string::npos)
  {
  }

  hid_t phasesGid = H5Gopen(gid, EbsdLib::H5OINA::Phases.c_str(), H5P_DEFAULT);
  if(phasesGid < 0)
  {
    setErrorCode(-90007);
    setErrorMessage("H5OINAReader Error: Could not open Header/Phase HDF Group.");
    H5Gclose(gid);
    return getErrorCode();
  }
  sentinel.addGroupId(phasesGid);

  std::list<std::string> names;
  err = H5Utilities::getGroupObjects(phasesGid, H5Utilities::CustomHDFDataTypes::Group, names);
  if(err < 0 || names.empty())
  {
    setErrorCode(-90009);
    setErrorMessage("H5OINAReader Error: There were no Phase groups present in the HDF5 file");
    H5Gclose(phasesGid);
    H5Gclose(gid);
    return getErrorCode();
  }
  // m_Phases.clear();
  std::vector<CtfPhase::Pointer> phaseVector;

  for(const auto& phaseGroupName : names)
  {
    hid_t pid = H5Gopen(phasesGid, phaseGroupName.c_str(), H5P_DEFAULT);

    CtfPhase::Pointer currentPhase = CtfPhase::New();
    currentPhase->setPhaseIndex(std::stoi(phaseGroupName));

    READ_PHASE_STRING_DATA("H5OINAReader", pid, EbsdLib::H5OINA::PhaseName, PhaseName, currentPhase)

    std::vector<float> latticeConstants;
    err = H5Support::H5Lite::readVectorDataset(pid, EbsdLib::H5OINA::LatticeDimensions, latticeConstants);

    std::vector<float> latticeAngles;
    err = H5Support::H5Lite::readVectorDataset(pid, EbsdLib::H5OINA::LatticeAngles, latticeAngles);

    currentPhase->setLatticeConstants({latticeConstants[0], latticeConstants[1], latticeConstants[2], latticeAngles[0], latticeAngles[1], latticeAngles[1]});

    int laueGroup = 0;
    err = H5Support::H5Lite::readScalarDataset(pid, EbsdLib::H5OINA::LaueGroup, laueGroup);
    currentPhase->setLaueGroup(static_cast<EbsdLib::Ctf::LaueGroupTable>(laueGroup));

    int spaceGroup = 0;
    err = H5Support::H5Lite::readScalarDataset(pid, EbsdLib::H5OINA::SpaceGroup, spaceGroup);
    currentPhase->setSpaceGroup(spaceGroup);

    phaseVector.push_back(currentPhase);
    err = H5Gclose(pid);
  }

  setPhaseVector(phaseVector);

  std::string completeHeader;
  setOriginalHeader(completeHeader);

  return getErrorCode();
}

/**
 * @brief
 * @tparam T
 */
template <typename T>
int32_t AllocateAndReadData(H5OINAReader* c, hid_t gid, const std::string& name, std::vector<T>& data)
{
  int32_t err = H5Support::H5Lite::readVectorDataset(gid, name, data);
  if(err < 0)
  {
    std::stringstream ss;
    ss << c->getNameOfClass() << " Error: Could not read from HDF5 file for array named " << name << "\n";
    c->setErrorCode(-902302);
    c->setErrorMessage(ss.str());
  }
  return err;
}

// -----------------------------------------------------------------------------
int H5OINAReader::readData(hid_t parId)
{
  int err = -1;

  // Initialize new pointers
  size_t totalDataRows = 0;

  size_t nColumns = getXCells();
  size_t nRows = getYCells();

  if(nRows < 1)
  {
    err = -200;
    setErrorMessage("H5OINAReader Error: The number of Rows was < 1.");
    setErrorCode(err);
    return err;
  }

  totalDataRows = nRows * nColumns; /* nCols = nOddCols;*/

  if(totalDataRows == 0)
  {
    setErrorCode(-90301);
    setErrorMessage("There is no data to read. NumRows or NumColumns is Zero (0)");
    return -301;
  }

  hid_t gid = H5Gopen(parId, EbsdLib::H5OINA::Data.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    setErrorMessage("H5OINAReader Error: Could not open 'Data' Group");
    setErrorCode(-90012);
    return -90012;
  }
  setNumberOfElements(totalDataRows);
  size_t numBytes = totalDataRows * sizeof(float);
  std::string sBuf;
  std::stringstream ss(sBuf);

  if(m_ArrayNames.empty() && !m_ReadAllArrays)
  {
    err = H5Gclose(gid);
    err = -90013;
    setErrorMessage("H5OINAReader Error: ReadAllArrays was FALSE and no other arrays were requested to be read.");
    setErrorCode(err);
    return err;
  }

  err = AllocateAndReadData<uint8_t>(this, gid, EbsdLib::H5OINA::BandContrast, m_BandContrast);
  if(err < 0)
  {
    return err;
  }
  err = AllocateAndReadData<uint8_t>(this, gid, EbsdLib::H5OINA::BandSlope, m_BandSlope);
  if(err < 0)
  {
    return err;
  }
  err = AllocateAndReadData<uint8_t>(this, gid, EbsdLib::H5OINA::Bands, m_Bands);
  if(err < 0)
  {
    return err;
  }
  err = AllocateAndReadData<uint8_t>(this, gid, EbsdLib::H5OINA::Error, m_Error);
  if(err < 0)
  {
    return err;
  }
  err = AllocateAndReadData<float>(this, gid, EbsdLib::H5OINA::Euler, m_Euler);
  if(err < 0)
  {
    return err;
  }
  err = AllocateAndReadData<float>(this, gid, EbsdLib::H5OINA::MeanAngularDeviation, m_MeanAngularDeviation);
  if(err < 0)
  {
    return err;
  }
  err = AllocateAndReadData<uint8_t>(this, gid, EbsdLib::H5OINA::Phase, m_Phase);
  if(err < 0)
  {
    return err;
  }
  err = AllocateAndReadData<float>(this, gid, EbsdLib::H5OINA::X, m_X);
  if(err < 0)
  {
    return err;
  }
  err = AllocateAndReadData<float>(this, gid, EbsdLib::H5OINA::Y, m_Y);
  if(err < 0)
  {
    return err;
  }
  //  if(m_ReadPatternData)
  //  {
  //    H5T_class_t type_class;
  //    std::vector<hsize_t> dims;
  //    size_t type_size = 0;
  //    err = H5Lite::getDatasetInfo(gid, EbsdLib::Ctf::PatternData, dims, type_class, type_size);
  //    if(err >= 0) // Only read the pattern data if the pattern data is available.
  //    {
  //      totalDataRows = 1; // Calculate the total number of elements to allocate for the pattern data
  //      for(unsigned long long dim : dims)
  //      {
  //        totalDataRows = totalDataRows * dim;
  //      }
  //      // Set the pattern dimensions
  //      m_PatternDims[0] = static_cast<int>(dims[1]);
  //      m_PatternDims[1] = static_cast<int>(dims[2]);
  //
  //      m_PatternData = this->allocateArray<uint8_t>(totalDataRows);
  //      err = H5Lite::readPointerDataset(gid, EbsdLib::Ctf::PatternData, m_PatternData);
  //    }
  //  }
  err = H5Gclose(gid);

  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5OINAReader::setArraysToRead(const std::set<std::string>& names)
{
  m_ArrayNames = names;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5OINAReader::readAllArrays(bool b)
{
  m_ReadAllArrays = b;
}

// -----------------------------------------------------------------------------
H5OINAReader::Pointer H5OINAReader::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
void H5OINAReader::setHDF5Path(const std::string& value)
{
  m_HDF5Path = value;
}

// -----------------------------------------------------------------------------
std::string H5OINAReader::getHDF5Path() const
{
  return m_HDF5Path;
}

// -----------------------------------------------------------------------------
std::string H5OINAReader::getNameOfClass() const
{
  return std::string("H5OINAReader");
}

// -----------------------------------------------------------------------------
std::string H5OINAReader::ClassName()
{
  return std::string("H5OINAReader");
}
