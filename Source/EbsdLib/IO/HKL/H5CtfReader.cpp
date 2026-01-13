/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
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

#include "H5CtfReader.h"

#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/IO/HKL/CtfConstants.h"

using namespace H5Support;

using namespace ebsdlib;

// -----------------------------------------------------------------------------
H5CtfReader::H5CtfReader()
: m_ReadAllArrays(true)
{
}

// -----------------------------------------------------------------------------
H5CtfReader::~H5CtfReader()
{
  if(m_PhaseCleanup)
  {
    deallocateArrayData<int32_t>(m_Phase);
    m_Phase = nullptr;
  }
  if(m_XCleanup)
  {
    deallocateArrayData<float>(m_X);
    m_X = nullptr;
  }
  if(m_YCleanup)
  {
    deallocateArrayData<float>(m_Y);
    m_Y = nullptr;
  }
  if(m_ZCleanup)
  {
    deallocateArrayData<float>(m_Z);
    m_Z = nullptr;
  }
  if(m_BandsCleanup)
  {
    deallocateArrayData<int32_t>(m_Bands);
    m_Bands = nullptr;
  }
  if(m_ErrorCleanup)
  {
    deallocateArrayData<int32_t>(m_Error);
    m_Error = nullptr;
  }
  if(m_Euler1Cleanup)
  {
    deallocateArrayData<float>(m_Euler1);
    m_Euler1 = nullptr;
  }
  if(m_Euler2Cleanup)
  {
    deallocateArrayData<float>(m_Euler2);
    m_Euler2 = nullptr;
  }
  if(m_Euler3Cleanup)
  {
    deallocateArrayData<float>(m_Euler3);
    m_Euler3 = nullptr;
  }
  if(m_MADCleanup)
  {
    deallocateArrayData<float>(m_MAD);
    m_MAD = nullptr;
  }
  if(m_BCCleanup)
  {
    deallocateArrayData<int32_t>(m_BC);
    m_BC = nullptr;
  }
  if(m_BSCleanup)
  {
    deallocateArrayData<int32_t>(m_BS);
    m_BS = nullptr;
  }
  if(m_GrainIndexCleanup)
  {
    deallocateArrayData<int32_t>(m_GrainIndex);
    m_GrainIndex = nullptr;
  }
  if(m_GrainRandomColourRCleanup)
  {
    deallocateArrayData<int32_t>(m_GrainRandomColourR);
    m_GrainRandomColourR = nullptr;
  }
  if(m_GrainRandomColourGCleanup)
  {
    deallocateArrayData<int32_t>(m_GrainRandomColourG);
    m_GrainRandomColourG = nullptr;
  }
  if(m_GrainRandomColourBCleanup)
  {
    deallocateArrayData<int32_t>(m_GrainRandomColourB);
    m_GrainRandomColourB = nullptr;
  }
}

// -----------------------------------------------------------------------------
int H5CtfReader::readHeaderOnly()
{
  int err = -1;
  if(m_HDF5Path.empty())
  {
    std::cout << "H5CtfReader Error: HDF5 Path is empty.";
    return -1;
  }

  hid_t fileId = H5Utilities::openFile(getFileName(), true);
  if(fileId < 0)
  {
    std::cout << "H5CtfReader Error: Could not open HDF5 file '" << getFileName() << "'";
    return -1;
  }

  H5ScopedFileSentinel sentinel(fileId, true);

  hid_t gid = H5Gopen(fileId, m_HDF5Path.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    std::cout << "H5CtfReader Error: Could not open path '" << m_HDF5Path << "'";
    err = H5Utilities::closeFile(fileId);
    return -1;
  }
  sentinel.addGroupId(gid);

  // Read all the header information
  err = readHeader(gid);
  return err;
}

// -----------------------------------------------------------------------------
int H5CtfReader::readFile()
{
  int err = -1;
  if(m_HDF5Path.empty())
  {
    std::cout << "H5CtfReader Error: HDF5 Path is empty.";
    return -1;
  }

  hid_t fileId = H5Utilities::openFile(getFileName(), true);
  if(fileId < 0)
  {
    std::cout << "H5CtfReader Error: Could not open HDF5 file '" << getFileName() << "'";
    return -1;
  }

  hid_t gid = H5Gopen(fileId, m_HDF5Path.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    std::cout << "H5CtfReader Error: Could not open path '" << m_HDF5Path << "'";
    err = H5Utilities::closeFile(fileId);
    return -1;
  }

  // Read all the header information
  err = readHeader(gid);

  // Read and transform data
  err = readData(gid);

  err = H5Gclose(gid);
  err = H5Utilities::closeFile(fileId);

  return err;
}

// -----------------------------------------------------------------------------
int H5CtfReader::readHeader(hid_t parId)
{
  std::string sBuf;
  std::stringstream ss(sBuf);
  int err = -1;
  hid_t gid = H5Gopen(parId, ebsdlib::H5Aztec::Header.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    std::cout << "H5CtfReader Error: Could not open 'Header' Group";
    return -1;
  }

  READ_EBSD_HEADER_STRING_DATA("H5CtfReader", CtfStringHeaderEntry, std::string, Prj, ebsdlib::Ctf::Prj, gid)
  READ_EBSD_HEADER_STRING_DATA("H5CtfReader", CtfStringHeaderEntry, std::string, Author, ebsdlib::Ctf::Author, gid)
  READ_EBSD_HEADER_STRING_DATA("H5CtfReader", CtfStringHeaderEntry, std::string, JobMode, ebsdlib::Ctf::JobMode, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfIntHeaderType, int, XCells, ebsdlib::Ctf::XCells, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfIntHeaderType, int, YCells, ebsdlib::Ctf::YCells, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfFloatHeaderType, float, XStep, ebsdlib::Ctf::XStep, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfFloatHeaderType, float, YStep, ebsdlib::Ctf::YStep, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfFloatHeaderType, float, AcqE1, ebsdlib::Ctf::AcqE1, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfFloatHeaderType, float, AcqE2, ebsdlib::Ctf::AcqE2, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfFloatHeaderType, float, AcqE3, ebsdlib::Ctf::AcqE3, gid)
  READ_EBSD_HEADER_STRING_DATA("H5CtfReader", CtfStringHeaderEntry, std::string, Euler, ebsdlib::Ctf::Euler, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfIntHeaderType, int, Mag, ebsdlib::Ctf::Mag, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfIntHeaderType, int, Coverage, ebsdlib::Ctf::Coverage, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfIntHeaderType, int, Device, ebsdlib::Ctf::Device, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfIntHeaderType, int, KV, ebsdlib::Ctf::KV, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfFloatHeaderType, float, TiltAngle, ebsdlib::Ctf::TiltAngle, gid)
  READ_EBSD_HEADER_DATA("H5CtfReader", CtfFloatHeaderType, float, TiltAxis, ebsdlib::Ctf::TiltAxis, gid)

  hid_t phasesGid = H5Gopen(gid, ebsdlib::H5Aztec::Phases.c_str(), H5P_DEFAULT);
  if(phasesGid < 0)
  {
    setErrorCode(-90007);
    setErrorMessage("H5CtfReader Error: Could not open Header/Phases HDF Group.");
    H5Gclose(gid);
    return -1;
  }

  std::list<std::string> names;
  err = H5Utilities::getGroupObjects(phasesGid, H5Utilities::CustomHDFDataTypes::Group, names);
  if(err < 0 || names.empty())
  {
    setErrorCode(-90009);
    setErrorMessage("H5CtfReader Error: There were no Phase groups present in the HDF5 file");
    H5Gclose(phasesGid);
    H5Gclose(gid);
    return -1;
  }
  m_Phases.clear();

  for(const auto& phaseGroupName : names)
  {
    hid_t pid = H5Gopen(phasesGid, phaseGroupName.c_str(), H5P_DEFAULT);
    CtfPhase::Pointer m_CurrentPhase = CtfPhase::New();

    READ_PHASE_HEADER_ARRAY("H5CtfReader", pid, float, ebsdlib::Ctf::LatticeConstants, LatticeConstants, m_CurrentPhase);
    READ_PHASE_STRING_DATA("H5CtfReader", pid, ebsdlib::Ctf::PhaseName, PhaseName, m_CurrentPhase)
    READ_PHASE_HEADER_DATA_CAST("H5CtfReader", pid, ebsdlib::Ctf::LaueGroupTable, int, ebsdlib::Ctf::LaueGroup, LaueGroup, m_CurrentPhase)
    READ_PHASE_HEADER_DATA_CAST("H5CtfReader", pid, int, int, ebsdlib::Ctf::SpaceGroup, SpaceGroup, m_CurrentPhase)
    READ_PHASE_STRING_DATA("H5CtfReader", pid, ebsdlib::Ctf::Internal1, Internal1, m_CurrentPhase)
    READ_PHASE_STRING_DATA("H5CtfReader", pid, ebsdlib::Ctf::Internal2, Internal2, m_CurrentPhase)
    READ_PHASE_STRING_DATA("H5CtfReader", pid, ebsdlib::Ctf::Comment, Comment, m_CurrentPhase)

    // For HKL Imports, the phase index is the HDF5 Group Name for this phase so
    // convert the phaseGroupName string variable into an integer
    int pIndex = std::stoi(phaseGroupName);
    m_CurrentPhase->setPhaseIndex(pIndex);
    m_Phases.push_back(m_CurrentPhase);
    err = H5Gclose(pid);
  }

  std::string completeHeader;
  err = H5Lite::readStringDataset(gid, ebsdlib::H5Aztec::OriginalHeader, completeHeader);
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
int H5CtfReader::readData(hid_t parId)
{
  int err = -1;

  size_t yCells = getYCells();
  size_t xCells = getXCells();
  size_t totalDataRows = yCells * xCells;
  if(totalDataRows == 0)
  {
    setErrorCode(-1);
    setErrorMessage(std::string("TotalDataRows = 0;"));
    return -1;
  }

  hid_t gid = H5Gopen(parId, ebsdlib::H5Aztec::Data.c_str(), H5P_DEFAULT);
  if(gid < 0)
  {
    setErrorMessage("H5CtfReader Error: Could not open 'Data' Group");
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
    setErrorMessage("H5CtfReader Error: ReadAllArrays was FALSE and no other arrays were requested to be read.");
    setErrorCode(err);
    return err;
  }

  // This data _SHOULD_ always be present in the file
  ANG_READER_ALLOCATE_AND_READ(Phase, ebsdlib::Ctf::Phase, int);
  ANG_READER_ALLOCATE_AND_READ(X, ebsdlib::Ctf::X, float);
  ANG_READER_ALLOCATE_AND_READ(Y, ebsdlib::Ctf::Y, float);
  ANG_READER_ALLOCATE_AND_READ(BandCount, ebsdlib::Ctf::Bands, int);
  ANG_READER_ALLOCATE_AND_READ(Error, ebsdlib::Ctf::Error, int);
  ANG_READER_ALLOCATE_AND_READ(Euler1, ebsdlib::Ctf::Euler1, float);
  ANG_READER_ALLOCATE_AND_READ(Euler2, ebsdlib::Ctf::Euler2, float);
  ANG_READER_ALLOCATE_AND_READ(Euler3, ebsdlib::Ctf::Euler3, float);
  ANG_READER_ALLOCATE_AND_READ(MeanAngularDeviation, ebsdlib::Ctf::MAD, float);
  ANG_READER_ALLOCATE_AND_READ(BandContrast, ebsdlib::Ctf::BC, int);
  ANG_READER_ALLOCATE_AND_READ(BandSlope, ebsdlib::Ctf::BS, int);
  // This data is optional in the file.
  ANG_READER_ALLOCATE_AND_READ(Z, ebsdlib::Ctf::Z, float);
  ANG_READER_ALLOCATE_AND_READ(GrainIndex, ebsdlib::Ctf::GrainIndex, int);
  ANG_READER_ALLOCATE_AND_READ(GrainRandomColourR, ebsdlib::Ctf::GrainRandomColourR, int);
  ANG_READER_ALLOCATE_AND_READ(GrainRandomColourG, ebsdlib::Ctf::GrainRandomColourG, int);
  ANG_READER_ALLOCATE_AND_READ(GrainRandomColourB, ebsdlib::Ctf::GrainRandomColourB, int);

  err = H5Gclose(gid);

  return err;
}

// -----------------------------------------------------------------------------
void H5CtfReader::setArraysToRead(const std::set<std::string>& names)
{
  m_ArrayNames = names;
}

// -----------------------------------------------------------------------------
void H5CtfReader::readAllArrays(bool b)
{
  m_ReadAllArrays = b;
}

// -----------------------------------------------------------------------------
H5CtfReader::Pointer H5CtfReader::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
void H5CtfReader::setHDF5Path(const std::string& value)
{
  m_HDF5Path = value;
}

// -----------------------------------------------------------------------------
std::string H5CtfReader::getHDF5Path() const
{
  return m_HDF5Path;
}

// -----------------------------------------------------------------------------
std::string H5CtfReader::getNameOfClass() const
{
  return std::string("H5CtfReader");
}

// -----------------------------------------------------------------------------
std::string H5CtfReader::ClassName()
{
  return std::string("H5CtfReader");
}
