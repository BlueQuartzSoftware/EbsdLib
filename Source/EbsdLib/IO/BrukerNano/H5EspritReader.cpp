/* ============================================================================
 * Copyright (c) 2019 BlueQuartz Software, LLC
 * All rights reserved.
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
 * Neither the names of any of the BlueQuartz Software contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "H5EspritReader.h"

#include <algorithm>
#include <vector>

#include <QtCore/QStringList>
#include <QtCore/QVector>
#include <QtCore/QtDebug>

#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"
#include "H5Support/QH5Lite.h"
#include "H5Support/QH5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/IO/BrukerNano/EspritPhase.h"

// -----------------------------------------------------------------------------
H5EspritReader::H5EspritReader()
: m_ReadPatternData(false)
, m_HDF5Path()
{
  m_HeaderMap.clear();
  // Initialize the map of header key to header value
  m_HeaderMap[EbsdLib::H5Esprit::CameraTilt] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::CameraTilt);
  m_HeaderMap[EbsdLib::H5Esprit::KV] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::KV);
  m_HeaderMap[EbsdLib::H5Esprit::MADMax] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::MADMax);
  m_HeaderMap[EbsdLib::H5Esprit::Magnification] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::Magnification);
  m_HeaderMap[EbsdLib::H5Esprit::MapStepFactor] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::MapStepFactor);
  m_HeaderMap[EbsdLib::H5Esprit::MaxRadonBandCount] = AngHeaderEntry<int32_t>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::MaxRadonBandCount);
  m_HeaderMap[EbsdLib::H5Esprit::MinIndexedBands] = AngHeaderEntry<int32_t>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::MinIndexedBands);
  m_HeaderMap[EbsdLib::H5Esprit::NCOLS] = AngHeaderEntry<int32_t>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::NCOLS);
  m_HeaderMap[EbsdLib::H5Esprit::NPoints] = AngHeaderEntry<int32_t>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::NPoints);
  m_HeaderMap[EbsdLib::H5Esprit::NROWS] = AngHeaderEntry<int32_t>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::NROWS);
  m_HeaderMap[EbsdLib::H5Esprit::PatternHeight] = AngHeaderEntry<int32_t>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::PatternHeight);
  m_HeaderMap[EbsdLib::H5Esprit::PatternWidth] = AngHeaderEntry<int32_t>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::PatternWidth);
  m_HeaderMap[EbsdLib::H5Esprit::PixelByteCount] = AngHeaderEntry<int32_t>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::PixelByteCount);
  m_HeaderMap[EbsdLib::H5Esprit::SEPixelSizeX] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::SEPixelSizeX);
  m_HeaderMap[EbsdLib::H5Esprit::SEPixelSizeY] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::SEPixelSizeY);
  m_HeaderMap[EbsdLib::H5Esprit::SampleTilt] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::SampleTilt);
  m_HeaderMap[EbsdLib::H5Esprit::TopClip] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::TopClip);
  m_HeaderMap[EbsdLib::H5Esprit::UnClippedPatternHeight] = AngHeaderEntry<int32_t>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::UnClippedPatternHeight);
  m_HeaderMap[EbsdLib::H5Esprit::WD] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::WD);
  m_HeaderMap[EbsdLib::H5Esprit::XSTEP] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::XSTEP);
  m_HeaderMap[EbsdLib::H5Esprit::YSTEP] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::YSTEP);
  m_HeaderMap[EbsdLib::H5Esprit::ZOffset] = AngHeaderEntry<double>::NewEbsdHeaderEntry(EbsdLib::H5Esprit::ZOffset);

  m_HeaderMap[EbsdLib::H5Esprit::GridType] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::H5Esprit::GridType);
  m_HeaderMap[EbsdLib::H5Esprit::OriginalFile] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::H5Esprit::OriginalFile);

  m_PatternDims[0] = -1;
  m_PatternDims[1] = -1;
}

// -----------------------------------------------------------------------------
H5EspritReader::~H5EspritReader()
{
  //  if(m_DDCleanup)
  //  {
  //    deallocateArrayData<EbsdLib::H5Esprit::DD_t>(m_DD);
  //    m_DD = nullptr;
  //  }
  if(m_MADCleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::MAD_t>(m_MAD);
    m_MAD = nullptr;
  }
  //  if(m_MADPhaseCleanup)
  //  {
  //    deallocateArrayData<EbsdLib::H5Esprit::MADPhase_t>(m_MADPhase);
  //    m_MADPhase = nullptr;
  //  }
  if(m_NIndexedBandsCleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::NIndexedBands_t>(m_NIndexedBands);
    m_NIndexedBands = nullptr;
  }
  //  if(m_PCXCleanup)
  //  {
  //    deallocateArrayData<EbsdLib::H5Esprit::PCX_t>(m_PCX);
  //    m_PCX = nullptr;
  //  }
  //  if(m_PCYCleanup)
  //  {
  //    deallocateArrayData<EbsdLib::H5Esprit::PCY_t>(m_PCY);
  //    m_PCY = nullptr;
  //  }
  if(m_PHICleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::PHI_t>(m_PHI);
    m_PHI = nullptr;
  }
  if(m_PhaseCleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::Phase_t>(m_Phase);
    m_Phase = nullptr;
  }
  if(m_RadonBandCountCleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::RadonBandCount_t>(m_RadonBandCount);
    m_RadonBandCount = nullptr;
  }
  if(m_RadonQualityCleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::RadonQuality_t>(m_RadonQuality);
    m_RadonQuality = nullptr;
  }
  if(m_XBEAMCleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::XBEAM_t>(m_XBEAM);
    m_XBEAM = nullptr;
  }
  if(m_YBEAMCleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::YBEAM_t>(m_YBEAM);
    m_YBEAM = nullptr;
  }
  //  if(m_XSAMPLECleanup)
  //  {
  //    deallocateArrayData<EbsdLib::H5Esprit::XSAMPLE_t>(m_XSAMPLE);
  //    m_XSAMPLE = nullptr;
  //  }
  //  if(m_YSAMPLECleanup)
  //  {
  //    deallocateArrayData<EbsdLib::H5Esprit::YSAMPLE_t>(m_YSAMPLE);
  //    m_YSAMPLE = nullptr;
  //  }
  if(m_phi1Cleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::phi1_t>(m_phi1);
    m_phi1 = nullptr;
  }
  if(m_phi2Cleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::phi2_t>(m_phi2);
    m_phi2 = nullptr;
  }

  if(m_PatternDataCleanup)
  {
    deallocateArrayData<EbsdLib::H5Esprit::RawPatterns_t>(m_PatternData);
    m_PatternData = nullptr;
  }
}

// -----------------------------------------------------------------------------
QVector<EspritPhase::Pointer> H5EspritReader::getPhaseVector() const
{
  return m_Phases;
}

// -----------------------------------------------------------------------------
int H5EspritReader::sanityCheckForOpening()
{
  int err = 0;
  setErrorCode(0);
  setErrorMessage(QString(""));

  if(m_HDF5Path.isEmpty())
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: HDF5 Path is empty.";
    setErrorCode(-1);
    setErrorMessage(str);
    return err;
  }

  if(getFileName().isEmpty())
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: Input filepath is empty";
    setErrorCode(-387);
    setErrorMessage(str);
    return err;
  }
  return err;
}

// -----------------------------------------------------------------------------
int H5EspritReader::readFile()
{
  int err = sanityCheckForOpening();
  if(getErrorCode() < 0)
  {
    return getErrorCode();
  }

  hid_t fileId = QH5Utilities::openFile(getFileName(), true);
  if(fileId < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: Could not open HDF5 file '" << getFileName() << "'";
    setErrorCode(-2);
    setErrorMessage(str);
    return err;
  }

  H5ScopedFileSentinel sentinel(&fileId, false);
  hid_t gid = H5Gopen(fileId, m_HDF5Path.toLatin1().data(), H5P_DEFAULT);
  if(gid < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: Could not open path '" << m_HDF5Path << "'";
    err = QH5Utilities::closeFile(fileId);
    setErrorCode(-90020);
    setErrorMessage(str);
    return getErrorCode();
  }
  sentinel.addGroupId(&gid);

  hid_t ebsdGid = H5Gopen(gid, EbsdLib::H5Esprit::EBSD.toLatin1().data(), H5P_DEFAULT);
  if(ebsdGid < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: Could not open 'EBSD' Group";
    setErrorCode(-90007);
    setErrorMessage(str);
    return getErrorCode();
  }
  sentinel.addGroupId(&ebsdGid);

  // Read all the header information
  err = readHeader(ebsdGid);
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: could not read header";
    return getErrorCode();
  }

  // Read data
  err = readData(ebsdGid);
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: could not read data. Internal Error code " << getErrorCode() << " generated.";
    return getErrorCode();
  }

  err = H5Gclose(ebsdGid);
  ebsdGid = -1;
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: could not close group id ";
    setErrorCode(-900023);
    setErrorMessage(str);
    return getErrorCode();
  }

  err = H5Gclose(gid);
  gid = -1;
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: could not close group id ";
    setErrorCode(-900023);
    setErrorMessage(str);
    return getErrorCode();
  }

  err = QH5Utilities::closeFile(fileId);
  fileId = -1;
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: could not close file";
    setErrorCode(-900024);
    setErrorMessage(str);
    return getErrorCode();
  }

  return getErrorCode();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EspritReader::readHeaderOnly()
{
  int err = sanityCheckForOpening();
  if(getErrorCode() < 0)
  {
    return getErrorCode();
  }
  hid_t fileId = QH5Utilities::openFile(getFileName(), true);
  if(fileId < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: Could not open HDF5 file '" << getFileName() << "'";
    setErrorCode(-2);
    setErrorMessage(str);
    return err;
  }

  H5ScopedFileSentinel sentinel(&fileId, false);
  hid_t gid = H5Gopen(fileId, m_HDF5Path.toLatin1().data(), H5P_DEFAULT);
  if(gid < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: Could not open path '" << m_HDF5Path << "'";
    err = QH5Utilities::closeFile(fileId);
    setErrorCode(-90020);
    setErrorMessage(str);
    return getErrorCode();
  }
  sentinel.addGroupId(&gid);

  hid_t ebsdGid = H5Gopen(gid, EbsdLib::H5Esprit::EBSD.toLatin1().data(), H5P_DEFAULT);
  if(ebsdGid < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: Could not open 'EBSD' Group";
    setErrorCode(-90007);
    setErrorMessage(str);
    return getErrorCode();
  }
  sentinel.addGroupId(&ebsdGid);

  // Read all the header information
  err = readHeader(ebsdGid);
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: could not read header";
    return getErrorCode();
  }

  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EspritReader::readScanNames(QStringList& names)
{
  if(getFileName().isEmpty())
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: Input filepath is empty";
    setErrorCode(-387);
    setErrorMessage(str);
    return getErrorCode();
  }

  hid_t fileId = QH5Utilities::openFile(getFileName().toLatin1().data(), true);
  if(fileId < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << " Error: Could not open HDF5 file '" << getFileName() << "'";
    setErrorCode(-20);
    setErrorMessage(str);
    names.clear();
    return getErrorCode();
  }
  H5ScopedFileSentinel sentinel(&fileId, false);

  int32_t err = QH5Utilities::getGroupObjects(fileId, H5Utilities::CustomHDFDataTypes::Group, names);
  setErrorCode(err);
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EspritReader::readHeader(hid_t parId)
{
  using AngHeaderDoubleType = AngHeaderEntry<double>;
  using AngHeaderIntType = AngHeaderEntry<int32_t>;

  m_PatternDims[0] = -1;
  m_PatternDims[1] = -2;

  int err = -1;

  hid_t gid = H5Gopen(parId, EbsdLib::H5Esprit::Header.toLatin1().data(), H5P_DEFAULT);
  if(gid < 0)
  {
    setErrorCode(-90008);
    setErrorMessage("H5EspritReader Error: Could not open 'Header' Group");
    return -1;
  }
  H5ScopedGroupSentinel sentinel(&gid, false);

  // ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::CameraTilt, gid, m_HeaderMap);
  ReadH5EbsdHeaderStringData<H5EspritReader, QString, AngStringHeaderEntry>(this, EbsdLib::H5Esprit::GridType, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::KV, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::MADMax, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::Magnification, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::MapStepFactor, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, int32_t, AngHeaderIntType>(this, EbsdLib::H5Esprit::MaxRadonBandCount, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, int32_t, AngHeaderIntType>(this, EbsdLib::H5Esprit::MinIndexedBands, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5EspritReader, int32_t, AngHeaderIntType>(this, EbsdLib::H5Esprit::NCOLS, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5EspritReader, int32_t, AngHeaderIntType>(this, EbsdLib::H5Esprit::NPoints, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5EspritReader, int32_t, AngHeaderIntType>(this, EbsdLib::H5Esprit::NROWS, gid, m_HeaderMap);
  ReadH5EbsdHeaderStringData<H5EspritReader, QString, AngStringHeaderEntry>(this, EbsdLib::H5Esprit::OriginalFile, gid, m_HeaderMap);
  // ReadH5EbsdHeaderData<H5EspritReader, int32_t, AngHeaderIntType>(this, EbsdLib::H5Esprit::PixelByteCount, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::SEPixelSizeX, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::SEPixelSizeY, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::SampleTilt, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::TopClip, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, int32_t, AngHeaderIntType>(this, EbsdLib::H5Esprit::UnClippedPatternHeight, gid, m_HeaderMap);
  //  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::WD, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::XSTEP, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::YSTEP, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5EspritReader, double, AngHeaderDoubleType>(this, EbsdLib::H5Esprit::ZOffset, gid, m_HeaderMap);

  HDF_ERROR_HANDLER_OFF
  // Read the Pattern Width - This may not exist
  bool patWidthExists = QH5Lite::datasetExists(gid, EbsdLib::H5Esprit::PatternWidth);
  if(patWidthExists)
  {
    ReadH5EbsdHeaderData<H5EspritReader, int32_t, AngHeaderIntType>(this, EbsdLib::H5Esprit::PatternWidth, gid, m_HeaderMap);
    m_PatternDims[1] = getPatternWidth();
  }
  else if(getReadPatternData() && !patWidthExists)
  {
    setErrorCode(-90016);
    setErrorMessage("H5EspritReader Error: Header/PatternWidth data set does not exist and ReadPatternData = true.");
    H5Gclose(gid);
    return getErrorCode();
  }

  // Read the Pattern Height - This may not exist
  bool patHeightExists = QH5Lite::datasetExists(gid, EbsdLib::H5Esprit::PatternHeight);
  if(patHeightExists)
  {
    ReadH5EbsdHeaderData<H5EspritReader, int32_t, AngHeaderIntType>(this, EbsdLib::H5Esprit::PatternHeight, gid, m_HeaderMap);
    m_PatternDims[0] = getPatternHeight();
  }
  else if(getReadPatternData() && !patHeightExists)
  {
    setErrorCode(-90017);
    setErrorMessage("H5EspritReader Error: Header/PatternHeight data set does not exist and ReadPatternData = true.");
    H5Gclose(gid);
    return getErrorCode();
  }

  bool patternDataExists = QH5Lite::datasetExists(parId, EbsdLib::H5Esprit::Data + "/" + EbsdLib::H5Esprit::RawPatterns);
  if(getReadPatternData() && !patternDataExists)
  {
    setErrorCode(-90018);
    setErrorMessage("H5EspritReader Error: Data/RawPatterns data set does not exist and ReadPatternData = true.");
    H5Gclose(gid);
    return getErrorCode();
  }
  HDF_ERROR_HANDLER_ON

  hid_t phasesGid = H5Gopen(gid, EbsdLib::H5Esprit::Phases.toLatin1().data(), H5P_DEFAULT);
  if(phasesGid < 0)
  {
    setErrorCode(-90007);
    setErrorMessage("H5EspritReader Error: Could not open Header/Phase HDF Group.");
    H5Gclose(gid);
    return getErrorCode();
  }
  sentinel.addGroupId(&phasesGid);

  QStringList names;
  err = QH5Utilities::getGroupObjects(phasesGid, H5Utilities::CustomHDFDataTypes::Group, names);
  if(err < 0 || names.empty())
  {
    setErrorCode(-90009);
    setErrorMessage("H5EspritReader Error: There were no Phase groups present in the HDF5 file");
    H5Gclose(phasesGid);
    H5Gclose(gid);
    return getErrorCode();
  }
  // m_Phases.clear();
  QVector<EspritPhase::Pointer> phaseVector;

  for(const auto& phaseGroupName : names)
  {
    hid_t pid = H5Gopen(phasesGid, phaseGroupName.toLatin1().data(), H5P_DEFAULT);

    EspritPhase::Pointer currentPhase = EspritPhase::New();
    currentPhase->setPhaseIndex(phaseGroupName.toInt());
    READ_PHASE_STRING_DATA("H5EspritReader", pid, EbsdLib::H5Esprit::Formula, Formula, currentPhase)

    READ_PHASE_HEADER_DATA("H5EspritReader", pid, int32_t, EbsdLib::H5Esprit::IT, IT, currentPhase)

    READ_PHASE_HEADER_ARRAY("H5EspritReader", pid, float, EbsdLib::H5Esprit::LatticeConstants, LatticeConstants, currentPhase);

    READ_PHASE_STRING_DATA("H5EspritReader", pid, EbsdLib::H5Esprit::Name, Name, currentPhase)

    READ_PHASE_HEADER_DATA("H5EspritReader", pid, int32_t, EbsdLib::H5Esprit::Setting, Setting, currentPhase)

    READ_PHASE_STRING_DATA("H5EspritReader", pid, EbsdLib::H5Esprit::SpaceGroup, SpaceGroup, currentPhase)

    phaseVector.push_back(currentPhase);
    err = H5Gclose(pid);
  }

  m_Phases = phaseVector;

  QString completeHeader;
  setOriginalHeader(completeHeader);

  err = H5Gclose(phasesGid);
  phasesGid = -1;
  err = H5Gclose(gid);
  gid = -1;

  return getErrorCode();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EspritReader::readData(hid_t parId)
{
  int err = -1;

  // Initialize new pointers
  size_t totalDataRows = 0;

  QString grid = getGrid();

  size_t nColumns = getNumColumns();
  size_t nRows = getNumRows();

  if(nRows < 1)
  {
    err = -200;
    setErrorMessage("H5EspritReader Error: The number of Rows was < 1.");
    setErrorCode(err);
    return err;
  }
  if(grid.startsWith(EbsdLib::H5Esprit::Isometric))
  {
    // if (nCols > 0) { numElements = nRows * nCols; }
    if(nColumns > 0)
    {
      totalDataRows = nRows * nColumns; /* nCols = nOddCols;*/
    }
    else
    {
      totalDataRows = 0;
    }
  }
  else // Grid was not set
  {
    setErrorCode(-90300);
    setErrorMessage("The Grid Type was not set in the file.");
    return -300;
  }

  if(totalDataRows == 0)
  {
    setErrorCode(-90301);
    setErrorMessage("There is no data to read. NumRows or NumColumns is Zero (0)");
    return -301;
  }

  hid_t gid = H5Gopen(parId, EbsdLib::H5Esprit::Data.toLatin1().data(), H5P_DEFAULT);
  if(gid < 0)
  {
    setErrorMessage("H5OIMReader Error: Could not open 'Data' Group");
    setErrorCode(-90012);
    return -90012;
  }
  setNumberOfElements(totalDataRows);
  size_t numBytes = totalDataRows * sizeof(float);
  QString sBuf;
  QTextStream ss(&sBuf);

  if(m_ArrayNames.empty() && !m_ReadAllArrays)
  {
    err = H5Gclose(gid);
    err = -90013;
    setErrorMessage("H5OIMReader Error: ReadAllArrays was FALSE and no other arrays were requested to be read.");
    setErrorCode(err);
    return err;
  }

  //  ANG_READER_ALLOCATE_AND_READ(DD, EbsdLib::H5Esprit::DD, EbsdLib::H5Esprit::DD_t);
  ANG_READER_ALLOCATE_AND_READ(MAD, EbsdLib::H5Esprit::MAD, EbsdLib::H5Esprit::MAD_t);
  // ANG_READER_ALLOCATE_AND_READ(MADPhase, EbsdLib::H5Esprit::MADPhase, EbsdLib::H5Esprit::MADPhase_t);
  ANG_READER_ALLOCATE_AND_READ(NIndexedBands, EbsdLib::H5Esprit::NIndexedBands, EbsdLib::H5Esprit::NIndexedBands_t);
  // ANG_READER_ALLOCATE_AND_READ(PCX, EbsdLib::H5Esprit::PCX, EbsdLib::H5Esprit::PCX_t);
  // ANG_READER_ALLOCATE_AND_READ(PCY, EbsdLib::H5Esprit::PCY, EbsdLib::H5Esprit::PCY_t);
  ANG_READER_ALLOCATE_AND_READ(PHI, EbsdLib::H5Esprit::PHI, EbsdLib::H5Esprit::PHI_t);
  ANG_READER_ALLOCATE_AND_READ(Phase, EbsdLib::H5Esprit::Phase, EbsdLib::H5Esprit::Phase_t);
  ANG_READER_ALLOCATE_AND_READ(RadonBandCount, EbsdLib::H5Esprit::RadonBandCount, EbsdLib::H5Esprit::RadonBandCount_t);
  ANG_READER_ALLOCATE_AND_READ(RadonQuality, EbsdLib::H5Esprit::RadonQuality, EbsdLib::H5Esprit::RadonQuality_t);
  ANG_READER_ALLOCATE_AND_READ(XBEAM, EbsdLib::H5Esprit::XBEAM, EbsdLib::H5Esprit::XBEAM_t);
  ANG_READER_ALLOCATE_AND_READ(YBEAM, EbsdLib::H5Esprit::YBEAM, EbsdLib::H5Esprit::YBEAM_t);
  // ANG_READER_ALLOCATE_AND_READ(XSAMPLE, EbsdLib::H5Esprit::XSAMPLE, EbsdLib::H5Esprit::XSAMPLE_t);
  // ANG_READER_ALLOCATE_AND_READ(YSAMPLE, EbsdLib::H5Esprit::YSAMPLE, EbsdLib::H5Esprit::YSAMPLE_t);
  ANG_READER_ALLOCATE_AND_READ(phi1, EbsdLib::H5Esprit::phi1, EbsdLib::H5Esprit::phi1_t);
  ANG_READER_ALLOCATE_AND_READ(phi2, EbsdLib::H5Esprit::phi2, EbsdLib::H5Esprit::phi2_t);

  if(m_ReadPatternData)
  {
    H5T_class_t type_class;
    QVector<hsize_t> dims;
    size_t type_size = 0;
    err = QH5Lite::getDatasetInfo(gid, EbsdLib::H5Esprit::RawPatterns, dims, type_class, type_size);
    if(err >= 0) // Only read the pattern data if the pattern data is available.
    {
      totalDataRows = std::accumulate(dims.begin(), dims.end(), 1, std::multiplies<size_t>());

      // Set the pattern dimensions
      m_PatternDims[0] = dims[1];
      m_PatternDims[1] = dims[2];

      m_PatternData = this->allocateArray<uint8_t>(totalDataRows);
      err = QH5Lite::readPointerDataset(gid, EbsdLib::H5Esprit::RawPatterns, m_PatternData);
    }
  }
  err = H5Gclose(gid);

  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5EspritReader::setArraysToRead(const QSet<QString>& names)
{
  m_ArrayNames = names;
  m_ReadAllArrays = m_ArrayNames.isEmpty();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5EspritReader::readAllArrays(bool b)
{
  m_ReadAllArrays = b;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EspritReader::getXDimension()
{
  return getNumColumns();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5EspritReader::setXDimension(int xdim)
{
  setNumColumns(xdim);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5EspritReader::getYDimension()
{
  return getNumRows();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5EspritReader::setYDimension(int ydim)
{
  setNumRows(ydim);
}

// -----------------------------------------------------------------------------
void H5EspritReader::releaseOwnership(const QString& name)
{
  //  if(name == EbsdLib::H5Esprit::DD)
  //  {
  //    m_DD = nullptr;
  //    m_DDCleanup = false;
  //  }
  if(name == EbsdLib::H5Esprit::MAD)
  {
    m_MAD = nullptr;
    m_MADCleanup = false;
  }
  //  if(name == EbsdLib::H5Esprit::MADPhase)
  //  {
  //    m_MADPhase = nullptr;
  //    m_MADPhaseCleanup = false;
  //  }
  if(name == EbsdLib::H5Esprit::NIndexedBands)
  {
    m_NIndexedBands = nullptr;
    m_NIndexedBandsCleanup = false;
  }
  //  if(name == EbsdLib::H5Esprit::PCX)
  //  {
  //    m_PCX = nullptr;
  //    m_PCXCleanup = false;
  //  }
  //  if(name == EbsdLib::H5Esprit::PCY)
  //  {
  //    m_PCY = nullptr;
  //    m_PCYCleanup = false;
  //  }
  if(name == EbsdLib::H5Esprit::PHI)
  {
    m_PHI = nullptr;
    m_PHICleanup = false;
  }
  if(name == EbsdLib::H5Esprit::Phase)
  {
    m_Phase = nullptr;
    m_PhaseCleanup = false;
  }
  if(name == EbsdLib::H5Esprit::RadonBandCount)
  {
    m_RadonBandCount = nullptr;
    m_RadonBandCountCleanup = false;
  }
  if(name == EbsdLib::H5Esprit::RadonQuality)
  {
    m_RadonQuality = nullptr;
    m_RadonQualityCleanup = false;
  }
  if(name == EbsdLib::H5Esprit::RawPatterns)
  {
    m_PatternData = nullptr;
    m_PatternDataCleanup = false;
  }
  if(name == EbsdLib::H5Esprit::XBEAM)
  {
    m_XBEAM = nullptr;
    m_XBEAMCleanup = false;
  }
  if(name == EbsdLib::H5Esprit::YBEAM)
  {
    m_YBEAM = nullptr;
    m_YBEAMCleanup = false;
  }
  //  if(name == EbsdLib::H5Esprit::XSAMPLE)
  //  {
  //    m_XSAMPLE = nullptr;
  //    m_XSAMPLECleanup = false;
  //  }
  //  if(name == EbsdLib::H5Esprit::YSAMPLE)
  //  {
  //    m_YSAMPLE = nullptr;
  //    m_YSAMPLECleanup = false;
  //  }
  if(name == EbsdLib::H5Esprit::phi1)
  {
    m_phi1 = nullptr;
    m_phi1Cleanup = false;
  }
  if(name == EbsdLib::H5Esprit::phi2)
  {
    m_phi2 = nullptr;
    m_phi2Cleanup = false;
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void* H5EspritReader::getPointerByName(const QString& featureName)
{
  //  if(featureName == EbsdLib::H5Esprit::DD)
  //  {
  //    return static_cast<void*>(m_DD);
  //  }
  if(featureName == EbsdLib::H5Esprit::MAD)
  {
    return static_cast<void*>(m_MAD);
  }
  //  if(featureName == EbsdLib::H5Esprit::MADPhase)
  //  {
  //    return static_cast<void*>(m_MADPhase);
  //  }
  if(featureName == EbsdLib::H5Esprit::NIndexedBands)
  {
    return static_cast<void*>(m_NIndexedBands);
  }
  //  if(featureName == EbsdLib::H5Esprit::PCX)
  //  {
  //    return static_cast<void*>(m_PCX);
  //  }
  //  if(featureName == EbsdLib::H5Esprit::PCY)
  //  {
  //    return static_cast<void*>(m_PCY);
  //  }
  if(featureName == EbsdLib::H5Esprit::PHI)
  {
    return static_cast<void*>(m_PHI);
  }
  if(featureName == EbsdLib::H5Esprit::Phase)
  {
    return static_cast<void*>(m_Phase);
  }
  if(featureName == EbsdLib::H5Esprit::RadonBandCount)
  {
    return static_cast<void*>(m_RadonBandCount);
  }
  if(featureName == EbsdLib::H5Esprit::RadonQuality)
  {
    return static_cast<void*>(m_RadonQuality);
  }
  if(featureName == EbsdLib::H5Esprit::RawPatterns)
  {
    return static_cast<void*>(m_PatternData);
  }
  if(featureName == EbsdLib::H5Esprit::XBEAM)
  {
    return static_cast<void*>(m_XBEAM);
  }
  if(featureName == EbsdLib::H5Esprit::YBEAM)
  {
    return static_cast<void*>(m_YBEAM);
  }
  //  if(featureName == EbsdLib::H5Esprit::XSAMPLE)
  //  {
  //    return static_cast<void*>(m_XSAMPLE);
  //  }
  //  if(featureName == EbsdLib::H5Esprit::YSAMPLE)
  //  {
  //    return static_cast<void*>(m_YSAMPLE);
  //  }
  if(featureName == EbsdLib::H5Esprit::phi1)
  {
    return static_cast<void*>(m_phi1);
  }
  if(featureName == EbsdLib::H5Esprit::phi2)
  {
    return static_cast<void*>(m_phi2);
  }

  return nullptr;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::NumericTypes::Type H5EspritReader::getPointerType(const QString& featureName)
{

  //  if(featureName == EbsdLib::H5Esprit::DD)
  //  {
  //    return EbsdLib::NumericTypes::Type::Float;
  //  }
  if(featureName == EbsdLib::H5Esprit::MAD)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  //  if(featureName == EbsdLib::H5Esprit::MADPhase)
  //  {
  //    return EbsdLib::NumericTypes::Type::Int32;
  //  }
  if(featureName == EbsdLib::H5Esprit::NIndexedBands)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  //  if(featureName == EbsdLib::H5Esprit::PCX)
  //  {
  //    return EbsdLib::NumericTypes::Type::Float;
  //  }
  //  if(featureName == EbsdLib::H5Esprit::PCY)
  //  {
  //    return EbsdLib::NumericTypes::Type::Float;
  //  }
  if(featureName == EbsdLib::H5Esprit::PHI)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::H5Esprit::Phase)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::H5Esprit::RadonBandCount)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::H5Esprit::RadonQuality)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::H5Esprit::XBEAM)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::H5Esprit::YBEAM)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  //  if(featureName == EbsdLib::H5Esprit::XSAMPLE)
  //  {
  //    return EbsdLib::NumericTypes::Type::Float;
  //  }
  //  if(featureName == EbsdLib::H5Esprit::YSAMPLE)
  //  {
  //    return EbsdLib::NumericTypes::Type::Float;
  //  }
  if(featureName == EbsdLib::H5Esprit::phi1)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::H5Esprit::phi2)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }

  if(featureName == EbsdLib::H5Esprit::RawPatterns)
  {
    return EbsdLib::NumericTypes::Type::UInt8;
  }

  return EbsdLib::NumericTypes::Type::UnknownNumType;
}

// -----------------------------------------------------------------------------
H5EspritReader::Pointer H5EspritReader::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
void H5EspritReader::setHDF5Path(const QString& value)
{
  m_HDF5Path = value;
}

// -----------------------------------------------------------------------------
QString H5EspritReader::getHDF5Path() const
{
  return m_HDF5Path;
}

// -----------------------------------------------------------------------------
QString H5EspritReader::getNameOfClass() const
{
  return QString("H5EspritReader");
}

// -----------------------------------------------------------------------------
QString H5EspritReader::ClassName()
{
  return QString("H5EspritReader");
}
