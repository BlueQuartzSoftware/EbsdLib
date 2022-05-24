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

#include "H5AngReader.h"

#include <vector>

#include <iostream>
//<====== REPLACE std::list<std::string> with an Alias from a global header
#include <vector>

#include "AngConstants.h"

#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdMacros.h"

using namespace H5Support;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
H5AngReader::H5AngReader()
: m_ReadAllArrays(true)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
H5AngReader::~H5AngReader() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5AngReader::readFile()
{
  int err = -1;
  if(m_HDF5Path.empty())
  {
    std::cout << "H5AngReader Error: HDF5 Path is empty.";
    return err;
  }

  hid_t fileId = H5Utilities::openFile(getFileName(), true);
  if(fileId < 0)
  {
    std::cout << "H5AngReader Error: Could not open HDF5 file '" << getFileName() << "'";
    return err;
  }

  hid_t gid = H5Gopen(fileId, m_HDF5Path.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    std::cout << "H5AngReader Error: Could not open path '" << m_HDF5Path << "'";
    err = H5Utilities::closeFile(fileId);
    return -1;
  }

  // Read all the header information
  err = readHeader(gid);
  if(err < 0)
  {
    std::cout << "H5AngReader Error: could not read header";
    err = H5Utilities::closeFile(fileId);
    return err;
  }

  // Read and transform data
  err = readData(gid);
  if(err < 0)
  {
    std::cout << "H5AngReader Error: could not read data";
    err = H5Utilities::closeFile(fileId);
    return err;
  }

  err = H5Gclose(gid);
  if(err < 0)
  {
    std::cout << "H5AngReader Error: could not close group id ";
    err = H5Utilities::closeFile(fileId);
    return err;
  }

  err = H5Utilities::closeFile(fileId);
  if(err < 0)
  {
    std::cout << "H5AngReader Error: could not close file";
    err = H5Utilities::closeFile(fileId);
    return err;
  }

  return getErrorCode();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5AngReader::readHeaderOnly()
{
  int err = -1;
  if(m_HDF5Path.empty())
  {
    std::cout << "H5AngReader Error: HDF5 Path is empty.";
    return -1;
  }

  hid_t fileId = H5Utilities::openFile(getFileName().c_str(), true);
  if(fileId < 0)
  {
    std::cout << "H5AngReader Error: Could not open HDF5 file '" << getFileName() << "'";
    return -1;
  }

  H5ScopedFileSentinel sentinel(fileId, true);

  hid_t gid = H5Gopen(fileId, m_HDF5Path.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    std::cout << "H5AngReader Error: Could not open path '" << m_HDF5Path << "'";
    err = H5Utilities::closeFile(fileId);
    return -1;
  }
  sentinel.addGroupId(gid);

  // Read all the header information
  err = readHeader(gid);
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5AngReader::readHeader(hid_t parId)
{
  int err = -1;
  hid_t gid = H5Gopen(parId, EbsdLib::H5OIM::Header.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    setErrorCode(-90008);
    setErrorMessage("H5AngReader Error: Could not open 'Header' Group");
    return -1;
  }

  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<float>, float, TEMPIXPerUM, EbsdLib::Ang::TEMPIXPerUM, gid)
  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<float>, float, XStar, EbsdLib::Ang::XStar, gid)
  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<float>, float, YStar, EbsdLib::Ang::YStar, gid)
  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<float>, float, ZStar, EbsdLib::Ang::ZStar, gid)
  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<float>, float, WorkingDistance, EbsdLib::Ang::WorkingDistance, gid)
  READ_EBSD_HEADER_STRING_DATA("H5AngReader", AngStringHeaderEntry, std::string, Grid, EbsdLib::Ang::Grid, gid)
  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<float>, float, XStep, EbsdLib::Ang::XStep, gid)
  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<float>, float, YStep, EbsdLib::Ang::YStep, gid)
  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<int>, int, NumOddCols, EbsdLib::Ang::NColsOdd, gid)
  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<int>, int, NumEvenCols, EbsdLib::Ang::NColsEven, gid)
  READ_EBSD_HEADER_DATA("H5AngReader", AngHeaderEntry<int>, int, NumRows, EbsdLib::Ang::NRows, gid)
  READ_EBSD_HEADER_STRING_DATA("H5AngReader", AngStringHeaderEntry, std::string, OIMOperator, EbsdLib::Ang::OPERATOR, gid)
  READ_EBSD_HEADER_STRING_DATA("H5AngReader", AngStringHeaderEntry, std::string, SampleID, EbsdLib::Ang::SAMPLEID, gid)
  READ_EBSD_HEADER_STRING_DATA("H5AngReader", AngStringHeaderEntry, std::string, SCANID, EbsdLib::Ang::SCANID, gid)

  hid_t phasesGid = H5Gopen(gid, EbsdLib::H5OIM::Phases.c_str(), H5P_DEFAULT);
  if(phasesGid < 0)
  {
    setErrorCode(-90007);
    setErrorMessage("H5AngReader Error: Could not open Header/Phases HDF Group.");
    H5Gclose(gid);
    return -1;
  }

  std::list<std::string> names;
  err = H5Utilities::getGroupObjects(phasesGid, H5Utilities::CustomHDFDataTypes::Group, names);
  if(err < 0 || names.empty())
  {
    setErrorCode(-90009);
    setErrorMessage("H5AngReader Error: There were no Phase groups present in the HDF5 file");
    H5Gclose(phasesGid);
    H5Gclose(gid);
    return -1;
  }
  m_Phases.clear();

  for(const auto& phaseGroupName : names)
  {
    hid_t pid = H5Gopen(phasesGid, phaseGroupName.c_str(), H5P_DEFAULT);
    AngPhase::Pointer currentPhase = AngPhase::New();

    READ_PHASE_HEADER_DATA("H5AngReader", pid, int, EbsdLib::Ang::Phase, PhaseIndex, currentPhase)
    READ_PHASE_STRING_DATA("H5AngReader", pid, EbsdLib::Ang::MaterialName, MaterialName, currentPhase)
    READ_PHASE_STRING_DATA("H5AngReader", pid, EbsdLib::Ang::Formula, Formula, currentPhase)
    // READ_PHASE_STRING_DATA("H5AngReader", pid, EbsdLib::Ang::Info, Info, m_CurrentPhase)
    READ_PHASE_HEADER_DATA_CAST("H5AngReader", pid, uint32_t, int, EbsdLib::Ang::Symmetry, Symmetry, currentPhase)
    READ_PHASE_HEADER_ARRAY("H5AngReader", pid, float, EbsdLib::Ang::LatticeConstants, LatticeConstants, currentPhase)
    READ_PHASE_HEADER_DATA("H5AngReader", pid, int, EbsdLib::Ang::NumberFamilies, NumberFamilies, currentPhase)

    if(currentPhase->getNumberFamilies() > 0)
    {
      hid_t hklGid = H5Gopen(pid, EbsdLib::Ang::HKLFamilies.c_str(), H5P_DEFAULT);
      // Only read the HKL Families if they are there. Trying to open the group will tell us if there
      // are any families to read

      err = readHKLFamilies(hklGid, currentPhase);
      err = H5Gclose(hklGid);
      if(getErrorCode() < 0)
      {
        err = H5Gclose(pid);
        H5Gclose(phasesGid);
        H5Gclose(gid);
        return -1;
      }
    }
    /* The 'Categories' header may actually be missing from certain types of .ang files */
    if(H5Lite::datasetExists(pid, EbsdLib::Ang::Categories))
    {
      READ_PHASE_HEADER_ARRAY("H5AngReader", pid, int, EbsdLib::Ang::Categories, Categories, currentPhase)
    }
    m_Phases.push_back(currentPhase);
    err = H5Gclose(pid);
  }

  std::string completeHeader;
  err = H5Lite::readStringDataset(gid, EbsdLib::H5OIM::OriginalHeader, completeHeader);
  if(err < 0)
  {
    setErrorCode(-90010);
    setErrorMessage("The dataset 'Original Header' was missing from the HDF5 file.");
  }
  setOriginalHeader(completeHeader);
  err = H5Gclose(phasesGid);
  err = H5Gclose(gid);
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5AngReader::readHKLFamilies(hid_t hklGid, const AngPhase::Pointer& phase)
{

  hid_t dataset;
  hid_t memtype;
  herr_t status = 1;
  HKLFamily_t data;
  std::vector<HKLFamily::Pointer> families;
  for(int i = 0; i < phase->getNumberFamilies(); ++i)
  {
    std::string dsetName = EbsdStringUtils::number(i);

    dataset = H5Dopen(hklGid, dsetName.c_str(), H5P_DEFAULT);

    memtype = H5Tcreate(H5T_COMPOUND, sizeof(HKLFamily_t));
    status = H5Tinsert(memtype, "H", HOFFSET(HKLFamily_t, h), H5T_NATIVE_INT);
    status = H5Tinsert(memtype, "K", HOFFSET(HKLFamily_t, k), H5T_NATIVE_INT);
    status = H5Tinsert(memtype, "L", HOFFSET(HKLFamily_t, l), H5T_NATIVE_INT);
    status = H5Tinsert(memtype, "Solution 1", HOFFSET(HKLFamily_t, s1), H5T_NATIVE_CHAR);
    status = H5Tinsert(memtype, "Diffraction Intensity", HOFFSET(HKLFamily_t, diffractionIntensity), H5T_NATIVE_FLOAT);
    status = H5Tinsert(memtype, "Solution 2", HOFFSET(HKLFamily_t, s2), H5T_NATIVE_CHAR);

    status = H5Dread(dataset, memtype, H5S_ALL, H5S_ALL, H5P_DEFAULT, (void*)(&data));
    if(status < 0)
    {
      setErrorCode(-90011);
      std::stringstream ss;
      ss << "H5AngReader Error: Could not read the HKLFamily data for family number " << i;
      setErrorMessage(ss.str());
      break;
    }
    status = H5Dclose(dataset); // Close the data set
    status = H5Tclose(memtype);
    HKLFamily::Pointer f = HKLFamily::New();
    f->copyFromStruct(&data);
    families.push_back(f);
  }
  phase->setHKLFamilies(families);
  return status;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5AngReader::readData(hid_t parId)
{
  int err = -1;

  size_t totalDataRows = 0;

  std::string grid = getGrid();

  size_t nOddCols = getNumOddCols();
  size_t nEvenCols = getNumEvenCols();
  size_t nRows = getNumRows();

  if(nRows < 1)
  {
    err = -200;
    setErrorMessage("H5AngReader Error: The number of Rows was < 1.");
    setErrorCode(err);
    return err;
  }
  if(grid.find(EbsdLib::Ang::SquareGrid) == 0)
  {
    // if (nCols > 0) { numElements = nRows * nCols; }
    if(nOddCols > 0)
    {
      totalDataRows = nRows * nOddCols; /* nCols = nOddCols;*/
    }
    else if(nEvenCols > 0)
    {
      totalDataRows = nRows * nEvenCols; /* nCols = nEvenCols; */
    }
    else
    {
      totalDataRows = 0;
    }
  }
  else if(grid.find(EbsdLib::Ang::HexGrid) == 0)
  {
    setErrorCode(-90400);
    setErrorMessage("Ang Files with Hex Grids Are NOT currently supported. Please convert them to Square Grid files first");
    return -400;
  }
  else // Grid was not set
  {
    setErrorCode(-90300);
    setErrorMessage("The Grid Type was not set in the file.");
    return -300;
  }

  hid_t gid = H5Gopen(parId, EbsdLib::H5OIM::Data.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    setErrorMessage("H5AngReader Error: Could not open 'Data' Group");
    setErrorCode(-90012);
    return getErrorCode();
  }
  setNumberOfElements(totalDataRows);
  size_t numBytes = totalDataRows * sizeof(float);
  std::string sBuf;
  std::stringstream ss(sBuf);

  if(m_ArrayNames.empty() && !m_ReadAllArrays)
  {
    err = H5Gclose(gid);
    err = -90013;
    setErrorMessage("H5AngReader Error: ReadAllArrays was FALSE and no other arrays were requested to be read.");
    setErrorCode(err);
    return err;
  }

  // Initialize new pointers
  ANG_READER_ALLOCATE_AND_READ(Phi1, EbsdLib::Ang::Phi1, float);
  ANG_READER_ALLOCATE_AND_READ(Phi, EbsdLib::Ang::Phi, float);
  ANG_READER_ALLOCATE_AND_READ(Phi2, EbsdLib::Ang::Phi2, float);
  ANG_READER_ALLOCATE_AND_READ(ImageQuality, EbsdLib::Ang::ImageQuality, float);
  ANG_READER_ALLOCATE_AND_READ(ConfidenceIndex, EbsdLib::Ang::ConfidenceIndex, float);
  ANG_READER_ALLOCATE_AND_READ(PhaseData, EbsdLib::Ang::PhaseData, int);
  ANG_READER_ALLOCATE_AND_READ(XPosition, EbsdLib::Ang::XPosition, float);
  ANG_READER_ALLOCATE_AND_READ(YPosition, EbsdLib::Ang::YPosition, float);
  ANG_READER_ALLOCATE_AND_READ(Fit, EbsdLib::Ang::Fit, float);
  if(err < 0)
  {
    setNumFeatures(9);
  }

  ANG_READER_ALLOCATE_AND_READ(SEMSignal, EbsdLib::Ang::SEMSignal, float);
  if(err < 0)
  {
    setNumFeatures(8);
  }

  err = H5Gclose(gid);

  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5AngReader::setArraysToRead(const std::set<std::string>& names)
{
  m_ArrayNames = names;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5AngReader::readAllArrays(bool b)
{
  m_ReadAllArrays = b;
}

// -----------------------------------------------------------------------------
H5AngReader::Pointer H5AngReader::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
void H5AngReader::setHDF5Path(const std::string& value)
{
  m_HDF5Path = value;
}

// -----------------------------------------------------------------------------
std::string H5AngReader::getHDF5Path() const
{
  return m_HDF5Path;
}

// ----------------------------------------------------------------------------
std::string H5AngReader::getNameOfClass() const
{
  return std::string("H5AngReader");
}

// -----------------------------------------------------------------------------
std::string H5AngReader::ClassName()
{
  return std::string("H5AngReader");
}
