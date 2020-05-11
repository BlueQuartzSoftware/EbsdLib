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

#include "H5OIMReader.h"

#include <vector>

#include <QtCore/QStringList>
#include <QtCore/QVector>
#include <QtCore/QtDebug>

#include "AngConstants.h"

#include "H5Support/H5Utilities.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/QH5Lite.h"
#include "H5Support/QH5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdMacros.h"

#if defined(H5Support_NAMESPACE)
using namespace H5Support_NAMESPACE;
#endif

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
H5OIMReader::H5OIMReader()
{
  // Initialize the map of header key to header value
  m_HeaderMap[EbsdLib::Ang::TEMPIXPerUM] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::TEMPIXPerUM);
  m_HeaderMap[EbsdLib::Ang::XStar] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::XStar);
  m_HeaderMap[EbsdLib::Ang::YStar] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::YStar);
  m_HeaderMap[EbsdLib::Ang::ZStar] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::ZStar);
  m_HeaderMap[EbsdLib::Ang::Working_Distance] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::Working_Distance);
  m_HeaderMap[EbsdLib::Ang::GridType] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::GridType);
  m_HeaderMap[EbsdLib::Ang::StepX] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::StepX);
  m_HeaderMap[EbsdLib::Ang::StepY] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::StepY);
  //  m_HeaderMap[EbsdLib::Ang::ZStep] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::ZStep); // NOT actually in the file>::NewEbsdHeaderEntry(); , but may be needed
  //  m_HeaderMap[EbsdLib::Ang::ZPos] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::ZPos); // NOT actually in the file>::NewEbsdHeaderEntry(); , but may be needed
  //  m_HeaderMap[EbsdLib::Ang::ZMax] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::ZMax); // NOT actually in the file>::NewEbsdHeaderEntry(); , but may be needed
  m_HeaderMap[EbsdLib::Ang::nColumns] = AngHeaderEntry<int>::NewEbsdHeaderEntry(EbsdLib::Ang::nColumns);
  m_HeaderMap[EbsdLib::Ang::nRows] = AngHeaderEntry<int>::NewEbsdHeaderEntry(EbsdLib::Ang::nRows);
  m_HeaderMap[EbsdLib::Ang::Operator] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::Operator);
  m_HeaderMap[EbsdLib::Ang::SampleID] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::SampleID);
  m_HeaderMap[EbsdLib::Ang::ScanID] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::ScanID);

  m_HeaderMap[EbsdLib::Ang::PatternWidth] = AngHeaderEntry<int>::NewEbsdHeaderEntry(EbsdLib::Ang::PatternWidth);
  m_HeaderMap[EbsdLib::Ang::PatternHeight] = AngHeaderEntry<int>::NewEbsdHeaderEntry(EbsdLib::Ang::PatternHeight);

  m_PatternDims[0] = 0;
  m_PatternDims[1] = 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
H5OIMReader::~H5OIMReader()
{
  this->deallocateArrayData<uint8_t>(m_PatternData);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OIMReader::readFile()
{
  int err = -1;
  if(m_HDF5Path.isEmpty())
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: HDF5 Path is empty.";
    setErrorCode(-1);
    setErrorMessage(str);
    return err;
  }

  hid_t fileId = QH5Utilities::openFile(getFileName(), true);
  if(fileId < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: Could not open HDF5 file '" << getFileName() << "'";
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
    ss << getNameOfClass() << "Error: Could not open path '" << m_HDF5Path << "'";
    setErrorCode(-90020);
    setErrorMessage(str);
    return getErrorCode();
  }
  sentinel.addGroupId(&gid);

  hid_t ebsdGid = H5Gopen(gid, EbsdLib::H5OIM::EBSD.toLatin1().data(), H5P_DEFAULT);
  if(ebsdGid < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: Could not open 'EBSD' Group";
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
    ss << getNameOfClass() << "Error: could not read header";
    setErrorCode(-900021);
    setErrorMessage(str);
    return getErrorCode();
  }

  // Read data
  err = readData(ebsdGid);
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: could not read data. Internal Error code " << err << " generated.";
    setErrorCode(-900022);
    setErrorMessage(str);
    return getErrorCode();
  }

  err = H5Gclose(ebsdGid);
  ebsdGid = -1;
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: could not close group id ";
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
    ss << getNameOfClass() << "Error: could not close group id ";
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
    ss << getNameOfClass() << "Error: could not close file";
    setErrorCode(-900024);
    setErrorMessage(str);
    return getErrorCode();
  }

  return getErrorCode();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OIMReader::readHeaderOnly()
{
  int err = -1;

  hid_t fileId = QH5Utilities::openFile(getFileName().toLatin1().data(), true);
  if(fileId < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: Could not open HDF5 file '" << getFileName() << "'";
    setErrorCode(-10);
    setErrorMessage(str);
    return getErrorCode();
  }
  H5ScopedFileSentinel sentinel(&fileId, false);

  if(m_HDF5Path.isEmpty())
  {
    QStringList names;
    err = QH5Utilities::getGroupObjects(fileId, H5Utilities::CustomHDFDataTypes::Group, names);

    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error (Internal HDF5 Path is empty): The name of the scan was not specified. There are " << names.count() << " scans available. ";
    int nameCount = names.count();
    if(nameCount < 10)
    {
      ss << " The scan names are: ";
    }
    else
    {
      nameCount = 10;
      ss << " The first 10 scan names are: ";
    }
    for(int i = 0; i < nameCount; ++i)
    {
      ss << names.at(i) << "\n";
    }
    setErrorCode(-11);
    setErrorMessage(str);
    return getErrorCode();
  }

  hid_t gid = H5Gopen(fileId, m_HDF5Path.toLatin1().data(), H5P_DEFAULT);
  if(gid < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: Could not open path '" << m_HDF5Path << "'";
    err = QH5Utilities::closeFile(fileId);
    setErrorCode(-12);
    setErrorMessage(str);
    return getErrorCode();
  }
  sentinel.addGroupId(&gid);

  hid_t ebsdGid = H5Gopen(gid, EbsdLib::H5OIM::EBSD.toLatin1().data(), H5P_DEFAULT);
  if(ebsdGid < 0)
  {
    setErrorMessage("H5OIMReader Error: Could not open 'EBSD' Group");
    setErrorCode(-90007);
    return getErrorCode();
  }
  sentinel.addGroupId(&ebsdGid);

  err = readHeader(ebsdGid);

  err = H5Gclose(ebsdGid);
  ebsdGid = -1;
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: could not close group id ";
    err = QH5Utilities::closeFile(fileId);
    setErrorMessage(str);
    setErrorCode(-900023);
    return getErrorCode();
  }

  err = H5Gclose(gid);
  gid = -1;
  if(err < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: could not close group id ";
    err = QH5Utilities::closeFile(fileId);
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
    ss << getNameOfClass() << "Error: could not close file";
    err = QH5Utilities::closeFile(fileId);
    setErrorCode(-900024);
    setErrorMessage(str);
    return getErrorCode();
  }

  return getErrorCode();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OIMReader::readScanNames(QStringList& names)
{
  int err = -1;
  hid_t fileId = QH5Utilities::openFile(getFileName().toLatin1().data(), true);
  if(fileId < 0)
  {
    QString str;
    QTextStream ss(&str);
    ss << getNameOfClass() << "Error: Could not open HDF5 file '" << getFileName() << "'";
    setErrorCode(-20);
    setErrorMessage(str);
    names.clear();
    return getErrorCode();
  }
  H5ScopedFileSentinel sentinel(&fileId, false);

  err = QH5Utilities::getGroupObjects(fileId, H5Utilities::CustomHDFDataTypes::Group, names);
  setErrorCode(err);
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OIMReader::readHeader(hid_t parId)
{
  using AngHeaderFloatType = AngHeaderEntry<float>;
  using AngHeaderIntType = AngHeaderEntry<int>;
  int err = -1;

  hid_t gid = H5Gopen(parId, EbsdLib::H5OIM::Header.toLatin1().data(), H5P_DEFAULT);
  if(gid < 0)
  {
    setErrorCode(-90008);
    setErrorMessage("H5OIMReader Error: Could not open 'Header' Group");
    return -1;
  }
  H5ScopedGroupSentinel sentinel(&gid, false);

  // QString path = EbsdLib::H5OIM::PatternCenterCalibration + "/" + EbsdLib::Ang::XStar;
  hid_t patternCenterCalibrationGid = H5Gopen(gid, EbsdLib::H5OIM::PatternCenterCalibration.toLatin1().data(), H5P_DEFAULT);
  if(patternCenterCalibrationGid < 0)
  {
    setErrorCode(-90008);
    setErrorMessage("H5OIMReader Error: Could not open 'Pattern Center Calibration' Group");
    return -1;
  }
  sentinel.addGroupId(&patternCenterCalibrationGid);
  ReadH5EbsdHeaderData<H5OIMReader, float, AngHeaderFloatType>(this, EbsdLib::Ang::XStar, patternCenterCalibrationGid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5OIMReader, float, AngHeaderFloatType>(this, EbsdLib::Ang::YStar, patternCenterCalibrationGid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5OIMReader, float, AngHeaderFloatType>(this, EbsdLib::Ang::ZStar, patternCenterCalibrationGid, m_HeaderMap);

  ReadH5EbsdHeaderData<H5OIMReader, float, AngHeaderFloatType>(this, EbsdLib::Ang::Working_Distance, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5OIMReader, float, AngHeaderFloatType>(this, EbsdLib::Ang::StepX, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5OIMReader, float, AngHeaderFloatType>(this, EbsdLib::Ang::StepY, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5OIMReader, int, AngHeaderIntType>(this, EbsdLib::Ang::nColumns, gid, m_HeaderMap);
  ReadH5EbsdHeaderData<H5OIMReader, int, AngHeaderIntType>(this, EbsdLib::Ang::nRows, gid, m_HeaderMap);

  HDF_ERROR_HANDLER_OFF
  int value = 0;
  if(QH5Lite::datasetExists(gid, EbsdLib::Ang::PatternWidth))
  {
    // Read the Pattern Width - This may not exist
    err = QH5Lite::readScalarDataset(gid, EbsdLib::Ang::PatternWidth, value);
    EbsdHeaderEntry::Pointer p = m_HeaderMap[EbsdLib::Ang::PatternWidth];
    AngHeaderIntType::Pointer c = std::dynamic_pointer_cast<AngHeaderIntType>(p);
    c->setValue(value);
    m_PatternDims[1] = value;
  }

  // Read the Pattern Height - This may not exist
  value = 0;
  if(QH5Lite::datasetExists(gid, EbsdLib::Ang::PatternHeight))
  {
    err = QH5Lite::readScalarDataset(gid, EbsdLib::Ang::PatternHeight, value);
    EbsdHeaderEntry::Pointer p = m_HeaderMap[EbsdLib::Ang::PatternHeight];
    AngHeaderIntType::Pointer c = std::dynamic_pointer_cast<AngHeaderIntType>(p);
    c->setValue(value);
    m_PatternDims[0] = value;
  }
  HDF_ERROR_HANDLER_ON

  ReadH5EbsdHeaderStringData<H5OIMReader, QString, AngStringHeaderEntry>(this, EbsdLib::Ang::Operator, gid, m_HeaderMap);
  ReadH5EbsdHeaderStringData<H5OIMReader, QString, AngStringHeaderEntry>(this, EbsdLib::Ang::SampleID, gid, m_HeaderMap);
  ReadH5EbsdHeaderStringData<H5OIMReader, QString, AngStringHeaderEntry>(this, EbsdLib::Ang::ScanID, gid, m_HeaderMap);
  ReadH5EbsdHeaderStringData<H5OIMReader, QString, AngStringHeaderEntry>(this, EbsdLib::Ang::GridType, gid, m_HeaderMap);

  hid_t phasesGid = H5Gopen(gid, EbsdLib::H5OIM::Phase.toLatin1().data(), H5P_DEFAULT);
  if(phasesGid < 0)
  {
    setErrorCode(-90007);
    setErrorMessage("H5OIMReader Error: Could not open Header/Phase HDF Group.");
    H5Gclose(gid);
    return getErrorCode();
  }
  sentinel.addGroupId(&phasesGid);

  QStringList names;
  err = QH5Utilities::getGroupObjects(phasesGid, H5Utilities::CustomHDFDataTypes::Group, names);
  if(err < 0 || names.empty())
  {
    setErrorCode(-90009);
    setErrorMessage("H5OIMReader Error: There were no Phase groups present in the HDF5 file");
    H5Gclose(phasesGid);
    H5Gclose(gid);
    return getErrorCode();
  }
  // m_Phases.clear();
  QVector<AngPhase::Pointer> phaseVector;

  foreach(QString phaseGroupName, names)
  {
    hid_t pid = H5Gopen(phasesGid, phaseGroupName.toLatin1().data(), H5P_DEFAULT);

    AngPhase::Pointer currentPhase = AngPhase::New();
    currentPhase->setPhaseIndex(phaseGroupName.toInt());
    READ_PHASE_STRING_DATA("H5OIMReader", pid, EbsdLib::Ang::MaterialName, MaterialName, currentPhase)
    READ_PHASE_STRING_DATA("H5OIMReader", pid, EbsdLib::Ang::Formula, Formula, currentPhase)
    READ_PHASE_STRING_DATA("H5OIMReader", pid, EbsdLib::Ang::Info, Info, currentPhase)
    READ_PHASE_HEADER_DATA("H5OIMReader", pid, int32_t, EbsdLib::Ang::Symmetry, Symmetry, currentPhase)
    READ_PHASE_HEADER_DATA("H5OIMReader", pid, int32_t, EbsdLib::Ang::NumberFamilies, NumberFamilies, currentPhase)

    QVector<float> fillerValues(6, 0.0);
    currentPhase->setLatticeConstants(fillerValues);
    READ_PHASE_HEADER_DATA("H5OIMReader", pid, float, EbsdLib::Ang::LatticeConstantA, LatticeConstantA, currentPhase)
    READ_PHASE_HEADER_DATA("H5OIMReader", pid, float, EbsdLib::Ang::LatticeConstantB, LatticeConstantB, currentPhase)
    READ_PHASE_HEADER_DATA("H5OIMReader", pid, float, EbsdLib::Ang::LatticeConstantC, LatticeConstantC, currentPhase)
    READ_PHASE_HEADER_DATA("H5OIMReader", pid, float, EbsdLib::Ang::LatticeConstantAlpha, LatticeConstantAlpha, currentPhase)
    READ_PHASE_HEADER_DATA("H5OIMReader", pid, float, EbsdLib::Ang::LatticeConstantBeta, LatticeConstantBeta, currentPhase)
    READ_PHASE_HEADER_DATA("H5OIMReader", pid, float, EbsdLib::Ang::LatticeConstantGamma, LatticeConstantGamma, currentPhase)

    if(currentPhase->getNumberFamilies() > 0)
    {
      // hid_t hklGid = H5Gopen(pid, EbsdLib::Ang::HKLFamilies.toLatin1().data(), H5P_DEFAULT);
      // Only read the HKL Families if they are there. Trying to open the group will tell us if there
      // are any families to read

      err = readHKLFamilies(pid, currentPhase);
      if(getErrorCode() < 0)
      {
        err = H5Gclose(pid);
        return -1;
      }
    }
    /* The 'Categories' header may actually be missing from certain types of .ang files */
    if(QH5Lite::datasetExists(pid, EbsdLib::Ang::Categories))
    {
      READ_PHASE_HEADER_ARRAY("H5OIMReader", pid, int, EbsdLib::Ang::Categories, Categories, currentPhase)
    }
    phaseVector.push_back(currentPhase);
    err = H5Gclose(pid);
  }

  setPhaseVector(phaseVector);

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
int H5OIMReader::readHKLFamilies(hid_t hklGid, const AngPhase::Pointer& phase)
{

  herr_t status = 1;
  // HKLFamily_t data;
  std::vector<HKLFamily_t> data;
  QVector<HKLFamily::Pointer> families;

  // setup compound memory type
  hid_t memtype = H5Tcreate(H5T_COMPOUND, sizeof(HKLFamily_t));
  H5Tinsert(memtype, "H", HOFFSET(HKLFamily_t, h), H5T_NATIVE_INT);
  H5Tinsert(memtype, "K", HOFFSET(HKLFamily_t, k), H5T_NATIVE_INT);
  H5Tinsert(memtype, "L", HOFFSET(HKLFamily_t, l), H5T_NATIVE_INT);
  H5Tinsert(memtype, "Diffraction Intensity", HOFFSET(HKLFamily_t, diffractionIntensity), H5T_NATIVE_FLOAT);
  H5Tinsert(memtype, "Use in Indexing", HOFFSET(HKLFamily_t, s1), H5T_NATIVE_CHAR);
  H5Tinsert(memtype, "Show bands", HOFFSET(HKLFamily_t, s2), H5T_NATIVE_CHAR);

  // Create dataspace & dataset
  hid_t dataset = H5Dopen(hklGid, EbsdLib::Ang::HKL_Families.toLatin1().data(), H5P_DEFAULT);
  hid_t dataspace = H5Dget_space(dataset);
  int rank = H5Sget_simple_extent_ndims(dataspace);
  if(rank == 1)
  {
    hsize_t dimsFam[1];
    int nDims = H5Sget_simple_extent_dims(dataspace, dimsFam, nullptr);
    if(nDims > 0)
    {
      data.resize(dimsFam[0]);
      herr_t status = H5Dread(dataset, memtype, H5S_ALL, H5S_ALL, H5P_DEFAULT, (void*)(data.data()));
      if(status < 0)
      {
        setErrorCode(-90011);
        QString ss = QObject::tr("H5OIMReader Error: Could not read the HKLFamily data");
        setErrorMessage(ss);
        return getErrorCode();
      }
    }
  }

  for(int i = 0; i < phase->getNumberFamilies(); ++i)
  {
    HKLFamily::Pointer f = HKLFamily::New();
    HKLFamily_t* ptr = data.data() + i;
    f->copyFromStruct(ptr);
    families.push_back(f);
  }

  //// close resources
  H5Tclose(memtype);
  H5Sclose(dataspace);
  H5Dclose(dataset);

  phase->setHKLFamilies(families);
  return status;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OIMReader::readData(hid_t parId)
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
    setErrorMessage("H5OIMReader Error: The number of Rows was < 1.");
    setErrorCode(err);
    return err;
  }
  if(grid.startsWith(EbsdLib::Ang::SquareGrid))
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
  else if(grid.startsWith(EbsdLib::Ang::HexGrid))
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

  if(totalDataRows == 0)
  {
    setErrorCode(-90301);
    setErrorMessage("There is no data to read. NumRows or NumColumns is Zero (0)");
    return -301;
  }

  hid_t gid = H5Gopen(parId, EbsdLib::H5OIM::Data.toLatin1().data(), H5P_DEFAULT);
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

  ANG_READER_ALLOCATE_AND_READ(Phi1, EbsdLib::Ang::Phi1, float);
  ANG_READER_ALLOCATE_AND_READ(Phi, EbsdLib::Ang::Phi, float);
  ANG_READER_ALLOCATE_AND_READ(Phi2, EbsdLib::Ang::Phi2, float);
  ANG_READER_ALLOCATE_AND_READ(ImageQuality, EbsdLib::Ang::IQ, float);
  ANG_READER_ALLOCATE_AND_READ(ConfidenceIndex, EbsdLib::Ang::CI, float);
  ANG_READER_ALLOCATE_AND_READ(PhaseData, EbsdLib::Ang::Phase, int);
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

  if(m_ReadPatternData)
  {
    H5T_class_t type_class;
    QVector<hsize_t> dims;
    size_t type_size = 0;
    err = QH5Lite::getDatasetInfo(gid, EbsdLib::Ang::PatternData, dims, type_class, type_size);
    if(err >= 0) // Only read the pattern data if the pattern data is available.
    {
      totalDataRows = 1; // Calculate the total number of elements to allocate for the pattern data
      for(unsigned long long dim : dims)
      {
        totalDataRows = totalDataRows * dim;
      }
      // Set the pattern dimensions
      m_PatternDims[0] = dims[1];
      m_PatternDims[1] = dims[2];

      m_PatternData = this->allocateArray<uint8_t>(totalDataRows);
      err = QH5Lite::readPointerDataset(gid, EbsdLib::Ang::PatternData, m_PatternData);
    }
  }
  err = H5Gclose(gid);

  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5OIMReader::setArraysToRead(const QSet<QString>& names)
{
  m_ArrayNames = names;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5OIMReader::readAllArrays(bool b)
{
  m_ReadAllArrays = b;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OIMReader::getXDimension()
{
  return getNumColumns();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5OIMReader::setXDimension(int xdim)
{
  setNumColumns(xdim);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5OIMReader::getYDimension()
{
  return getNumRows();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5OIMReader::setYDimension(int ydim)
{
  setNumRows(ydim);
}

// -----------------------------------------------------------------------------
H5OIMReader::Pointer H5OIMReader::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
void H5OIMReader::setHDF5Path(const QString& value)
{
  m_HDF5Path = value;
}

// -----------------------------------------------------------------------------
QString H5OIMReader::getHDF5Path() const
{
  return m_HDF5Path;
}

// -----------------------------------------------------------------------------
QString H5OIMReader::getNameOfClass() const
{
  return QString("H5OIMReader");
}

// -----------------------------------------------------------------------------
QString H5OIMReader::ClassName()
{
  return QString("H5OIMReader");
}
