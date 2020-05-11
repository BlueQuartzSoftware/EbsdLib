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

#include "H5AngImporter.h"

#include "H5Support/QH5Lite.h"
#include "H5Support/QH5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/EbsdLibVersion.h"

#if defined(H5Support_NAMESPACE)
using namespace H5Support_NAMESPACE;
#endif

#define AIM_STRING QString

#define CHECK_FOR_CANCELED(AClass)                                                                                                                                                                     \
  if(m_Cancel == true)                                                                                                                                                                                 \
  {                                                                                                                                                                                                    \
    break;                                                                                                                                                                                             \
  }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
H5AngImporter::H5AngImporter()
: xDim(0)
, yDim(0)
, xRes(0)
, yRes(0)
, m_FileVersion(EbsdLib::H5OIM::FileVersion)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
H5AngImporter::~H5AngImporter() = default;

#define WRITE_ANG_HEADER_DATA(reader, m_msgType, prpty, key)                                                                                                                                           \
  {                                                                                                                                                                                                    \
    m_msgType t = reader.get##prpty();                                                                                                                                                                 \
    err = QH5Lite::writeScalarDataset(gid, key, t);                                                                                                                                                    \
    if(err < 0)                                                                                                                                                                                        \
    {                                                                                                                                                                                                  \
      ss.string()->clear();                                                                                                                                                                            \
      ss << "H5AngImporter Error: Could not write Ang Header value '" << t << "' to the HDF5 file with data set name '" << key << "'\n";                                                               \
      progressMessage(ss.string(), 100);                                                                                                                                                               \
      err = H5Gclose(gid);                                                                                                                                                                             \
      err = H5Gclose(angGroup);                                                                                                                                                                        \
      return -1;                                                                                                                                                                                       \
    }                                                                                                                                                                                                  \
  }

#define WRITE_ANG_HEADER_STRING_DATA(reader, m_msgType, prpty, key)                                                                                                                                    \
  {                                                                                                                                                                                                    \
    m_msgType t = reader.get##prpty();                                                                                                                                                                 \
    err = QH5Lite::writeStringDataset(gid, key, t);                                                                                                                                                    \
    if(err < 0)                                                                                                                                                                                        \
    {                                                                                                                                                                                                  \
      ss.string()->clear();                                                                                                                                                                            \
      ss << "H5AngImporter Error: Could not write Ang Header value '" << t << "' to the HDF5 file with data set name '" << key << "'\n";                                                               \
      progressMessage(ss.string(), 100);                                                                                                                                                               \
      err = H5Gclose(gid);                                                                                                                                                                             \
      err = H5Gclose(angGroup);                                                                                                                                                                        \
      return -1;                                                                                                                                                                                       \
    }                                                                                                                                                                                                  \
  }

#define WRITE_ANG_DATA_ARRAY(reader, m_msgType, gid, prpty, key)                                                                                                                                       \
  {                                                                                                                                                                                                    \
    m_msgType* dataPtr = reader.get##prpty##Pointer();                                                                                                                                                 \
    if(nullptr != dataPtr)                                                                                                                                                                             \
    {                                                                                                                                                                                                  \
      err = QH5Lite::writePointerDataset(gid, key, rank, dims, dataPtr);                                                                                                                               \
      if(err < 0)                                                                                                                                                                                      \
      {                                                                                                                                                                                                \
        ss.string()->clear();                                                                                                                                                                          \
        ss << "H5AngImporter Error: Could not write Ang Data array for '" << key << "' to the HDF5 file with data set name '" << key << "'\n";                                                         \
        progressMessage(ss.string(), 100);                                                                                                                                                             \
        err = H5Gclose(gid);                                                                                                                                                                           \
        err = H5Gclose(angGroup);                                                                                                                                                                      \
        return -1;                                                                                                                                                                                     \
      }                                                                                                                                                                                                \
    }                                                                                                                                                                                                  \
  }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5AngImporter::getDims(int64_t& x, int64_t& y)
{
  x = xDim;
  y = yDim;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5AngImporter::getSpacing(float& x, float& y)
{
  x = xRes;
  y = yRes;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5AngImporter::numberOfSlicesImported()
{
  return 1;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5AngImporter::importFile(hid_t fileId, int64_t z, const QString& angFile)
{
  herr_t err = -1;
  setCancel(false);
  setErrorCode(0);
  // setPipelineMessage("");
  QString streamBuf;
  QTextStream ss(&streamBuf);

  //  std::cout << "H5AngImporter: Importing " << angFile;
  AngReader reader;
  reader.setFileName(angFile);

  // Now actually read the file
  err = reader.readFile();

  // Check for errors
  if(err < 0)
  {
    if(err == -400)
    {
      ss << "H5AngImporter Error: HexGrid Files are not currently supported.";
    }
    else if(err == -300)
    {
      ss << "H5AngImporter Error: Grid was NOT set in the header.";
    }
    else if(err == -200)
    {
      ss << "H5AngImporter Error: There was no data in the file.";
    }
    else if(err == -100)
    {
      ss << "H5AngImporter Error: The Ang file could not be opened.'" << angFile << "'";
    }
    else if(reader.getXStep() == 0.0f)
    {
      ss << "H5AngImporter Error: X Step value equals 0.0. This is bad. Please check the validity of the ANG file.";
    }
    else if(reader.getYStep() == 0.0f)
    {
      ss << "H5AngImporter Error: Y Step value equals 0.0. This is bad. Please check the validity of the ANG file.";
    }
    else
    {
      ss << "H5AngImporter Error: Unknown error [" << err << "]";
    }
    // setPipelineMessage( *(ss.string()));

    setErrorCode(err);
    progressMessage(*(ss.string()), 100);
    return -1;
  }

  // Write the file Version number to the file
  {
    QVector<hsize_t> dims;
    H5T_class_t type_class;
    size_t type_size = 0;
    hid_t attr_type = -1;
    err = QH5Lite::getAttributeInfo(fileId, "/", EbsdLib::H5OIM::FileVersionStr, dims, type_class, type_size, attr_type);
    if(attr_type < 0) // The attr_type variable was never set which means the attribute was NOT there
    {
      // The file version does not exist so write it to the file
      err = QH5Lite::writeScalarAttribute(fileId, QString("/"), EbsdLib::H5OIM::FileVersionStr, m_FileVersion);
    }
    else
    {
      H5Aclose(attr_type);
    }

    err = QH5Lite::getAttributeInfo(fileId, "/", EbsdLib::H5OIM::EbsdLibVersionStr, dims, type_class, type_size, attr_type);
    if(attr_type < 0) // The attr_type variable was never set which means the attribute was NOT there
    {
      // The file version does not exist so write it to the file
      err = QH5Lite::writeStringAttribute(fileId, QString("/"), EbsdLib::H5OIM::EbsdLibVersionStr, EbsdLib::Version::Complete());
    }
    else
    {
      H5Aclose(attr_type);
    }
  }

  // Start creating the HDF5 group structures for this file
  hid_t angGroup = QH5Utilities::createGroup(fileId, QString::number(z));
  if(angGroup < 0)
  {
    ss.string()->clear();
    ss << "H5AngImporter Error: A Group for Z index " << z << " could not be created."
       << " Please check other error messages from the HDF5 library for possible reasons.";
    // setPipelineMessage( *(ss.string() ));
    setErrorCode(-500);
    return -1;
  }

  hid_t gid = QH5Utilities::createGroup(angGroup, EbsdLib::H5OIM::Header);
  if(gid < 0)
  {
    ss.string()->clear();
    ss << "H5AngImporter Error: The 'Header' Group for Z index " << z << " could not be created."
       << " Please check other error messages from the HDF5 library for possible reasons.";
    progressMessage(ss.string(), 100);
    err = H5Gclose(angGroup);
    // setPipelineMessage( *(ss.string()) );
    setErrorCode(-600);
    return -1;
  }
  WRITE_ANG_HEADER_DATA(reader, float, TEMpixPerum, EbsdLib::Ang::TEMPIXPerUM)
  WRITE_ANG_HEADER_DATA(reader, float, XStar, EbsdLib::Ang::XStar)
  WRITE_ANG_HEADER_DATA(reader, float, YStar, EbsdLib::Ang::YStar)
  WRITE_ANG_HEADER_DATA(reader, float, ZStar, EbsdLib::Ang::ZStar)
  WRITE_ANG_HEADER_DATA(reader, float, WorkingDistance, EbsdLib::Ang::WorkingDistance)

  hid_t phasesGid = QH5Utilities::createGroup(gid, EbsdLib::H5OIM::Phases);
  err = writePhaseData(reader, phasesGid);
  // Close this group
  err = H5Gclose(phasesGid);

  WRITE_ANG_HEADER_STRING_DATA(reader, QString, Grid, EbsdLib::Ang::Grid)
  WRITE_ANG_HEADER_DATA(reader, float, XStep, EbsdLib::Ang::XStep)
  xRes = reader.getXStep();
  WRITE_ANG_HEADER_DATA(reader, float, YStep, EbsdLib::Ang::YStep)
  yRes = reader.getYStep();
  WRITE_ANG_HEADER_DATA(reader, int, NumOddCols, EbsdLib::Ang::NColsOdd)
  WRITE_ANG_HEADER_DATA(reader, int, NumEvenCols, EbsdLib::Ang::NColsEven)
  xDim = reader.getNumEvenCols();
  WRITE_ANG_HEADER_DATA(reader, int, NumRows, EbsdLib::Ang::NRows)
  yDim = reader.getNumRows();
  WRITE_ANG_HEADER_STRING_DATA(reader, QString, OIMOperator, EbsdLib::Ang::OPERATOR)
  WRITE_ANG_HEADER_STRING_DATA(reader, QString, SampleID, EbsdLib::Ang::SAMPLEID)
  WRITE_ANG_HEADER_STRING_DATA(reader, QString, SCANID, EbsdLib::Ang::SCANID)

  QString angCompleteHeader = reader.getOriginalHeader();
  err = QH5Lite::writeStringDataset(gid, EbsdLib::H5OIM::OriginalHeader, angCompleteHeader);
  err = QH5Lite::writeStringDataset(gid, EbsdLib::H5OIM::OriginalFile, angFile);

  // Close the "Header" group
  err = H5Gclose(gid);

  // Create the "Data" group
  gid = QH5Utilities::createGroup(angGroup, EbsdLib::H5OIM::Data);
  if(gid < 0)
  {
    ss.string()->clear();
    ss << "H5AngImporter Error: The 'Data' Group for Z index " << z << " could not be created."
       << " Please check other error messages from the HDF5 library for possible reasons.\n";
    progressMessage(ss.string(), 100);
    err = H5Gclose(angGroup);
    // setPipelineMessage(*(ss.string()));
    setErrorCode(-700);
    return -1;
  }

  int32_t rank = 1;
  hsize_t dims[1] = {static_cast<hsize_t>(reader.getNumEvenCols() * reader.getNumRows())};

  WRITE_ANG_DATA_ARRAY(reader, float, gid, Phi1, EbsdLib::Ang::Phi1);
  WRITE_ANG_DATA_ARRAY(reader, float, gid, Phi, EbsdLib::Ang::Phi);
  WRITE_ANG_DATA_ARRAY(reader, float, gid, Phi2, EbsdLib::Ang::Phi2);
  WRITE_ANG_DATA_ARRAY(reader, float, gid, XPosition, EbsdLib::Ang::XPosition);
  WRITE_ANG_DATA_ARRAY(reader, float, gid, YPosition, EbsdLib::Ang::YPosition);
  WRITE_ANG_DATA_ARRAY(reader, float, gid, ImageQuality, EbsdLib::Ang::ImageQuality);
  WRITE_ANG_DATA_ARRAY(reader, float, gid, ConfidenceIndex, EbsdLib::Ang::ConfidenceIndex);
  WRITE_ANG_DATA_ARRAY(reader, int, gid, PhaseData, EbsdLib::Ang::PhaseData);
  WRITE_ANG_DATA_ARRAY(reader, float, gid, SEMSignal, EbsdLib::Ang::SEMSignal);
  WRITE_ANG_DATA_ARRAY(reader, float, gid, Fit, EbsdLib::Ang::Fit);
  // Close the "Data" group
  err = H5Gclose(gid);

  // Close the group for this file
  err = H5Gclose(angGroup);

  return err;
}

#define WRITE_PHASE_HEADER_DATA(reader, m_msgType, prpty, key)                                                                                                                                         \
  {                                                                                                                                                                                                    \
    m_msgType t = reader->get##prpty();                                                                                                                                                                \
    err = QH5Lite::writeScalarDataset(pid, key, t);                                                                                                                                                    \
    if(err < 0)                                                                                                                                                                                        \
    {                                                                                                                                                                                                  \
      ss.string()->clear();                                                                                                                                                                            \
      ss << "H5AngImporter Error: Could not write Ang Header value '" << t << "' to the HDF5 file with data set name '" << key << "'\n";                                                               \
      progressMessage(ss.string(), 100);                                                                                                                                                               \
      err = H5Gclose(pid);                                                                                                                                                                             \
      return -1;                                                                                                                                                                                       \
    }                                                                                                                                                                                                  \
  }

#define WRITE_PHASE_HEADER_STRING_DATA(reader, m_msgType, prpty, key)                                                                                                                                  \
  {                                                                                                                                                                                                    \
    m_msgType t = reader->get##prpty();                                                                                                                                                                \
    err = QH5Lite::writeStringDataset(pid, key, t);                                                                                                                                                    \
    if(err < 0)                                                                                                                                                                                        \
    {                                                                                                                                                                                                  \
      ss.string()->clear();                                                                                                                                                                            \
      ss << "H5AngImporter Error: Could not write Ang Header value '" << t << "' to the HDF5 file with data set name '" << key << "'\n";                                                               \
      progressMessage(ss.string(), 100);                                                                                                                                                               \
      err = H5Gclose(pid);                                                                                                                                                                             \
      return -1;                                                                                                                                                                                       \
    }                                                                                                                                                                                                  \
  }

#define WRITE_PHASE_DATA_ARRAY(reader, m_msgType, gid, prpty, key)                                                                                                                                     \
  {                                                                                                                                                                                                    \
    QVector<m_msgType> tempVar = reader->get##prpty();                                                                                                                                                 \
    dims[0] = tempVar.size();                                                                                                                                                                          \
    m_msgType* dataPtr = tempVar.data();                                                                                                                                                               \
    if(nullptr != dataPtr)                                                                                                                                                                             \
    {                                                                                                                                                                                                  \
      err = QH5Lite::writePointerDataset(pid, key, rank, dims, dataPtr);                                                                                                                               \
      if(err < 0)                                                                                                                                                                                      \
      {                                                                                                                                                                                                \
        ss.string()->clear();                                                                                                                                                                          \
        ss << "H5AngImporter Error: Could not write Ang Data array for '" << key << "' to the HDF5 file with data set name '" << key << "'\n";                                                         \
        progressMessage(ss.string(), 100);                                                                                                                                                             \
        err = H5Gclose(pid);                                                                                                                                                                           \
        return -1;                                                                                                                                                                                     \
      }                                                                                                                                                                                                \
    }                                                                                                                                                                                                  \
  }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5AngImporter::writePhaseData(AngReader& reader, hid_t phasesGid)
{
  QString sBuf;
  QTextStream ss(&sBuf);
  int err = 0;
  // int retErr = 0;
  int32_t rank = 1;
  hsize_t dims[1] = {0};
  QVector<AngPhase::Pointer> phases = reader.getPhaseVector();
  for(AngPhase::Pointer& phase : phases)
  {
    hid_t pid = QH5Utilities::createGroup(phasesGid, QString::number(phase->getPhaseIndex()));
    WRITE_PHASE_HEADER_DATA(phase, int, PhaseIndex, EbsdLib::Ang::Phase)
    WRITE_PHASE_HEADER_STRING_DATA(phase, QString, MaterialName, EbsdLib::Ang::MaterialName)
    WRITE_PHASE_HEADER_STRING_DATA(phase, QString, Formula, EbsdLib::Ang::Formula)
    WRITE_PHASE_HEADER_STRING_DATA(phase, QString, Info, EbsdLib::Ang::Info)
    WRITE_PHASE_HEADER_DATA(phase, int, Symmetry, EbsdLib::Ang::Symmetry)
    WRITE_PHASE_DATA_ARRAY(phase, float, pid, LatticeConstants, EbsdLib::Ang::LatticeConstants)
    WRITE_PHASE_HEADER_DATA(phase, int, NumberFamilies, EbsdLib::Ang::NumberFamilies)

    // Create a Group for the HKLFamilies
    if(phase->getNumberFamilies() > 0)
    {
      hid_t hklGid = QH5Utilities::createGroup(pid, EbsdLib::Ang::HKLFamilies);
      err = writeHKLFamilies(phase.get(), hklGid);
      if(err < 0)
      {
        ss.string()->clear();
        ss << "H5AngImporter Error: Could not write Ang HKL Families to the HDF5 file with data set name '" << EbsdLib::Ang::HKLFamilies << "'";
        progressMessage(ss.string(), 100);
        err = H5Gclose(hklGid);
        return -1;
      }
      err = H5Gclose(hklGid);
    }
    WRITE_PHASE_DATA_ARRAY(phase, int, pid, Categories, EbsdLib::Ang::Categories)
    err = H5Gclose(pid);
  }
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int H5AngImporter::writeHKLFamilies(AngPhase* p, hid_t hklGid)
{
  // int err = 0;
  hid_t memtype;
  hid_t space;
  hid_t dset;
  hsize_t dims[1] = {1};
  herr_t status = -1;
  int index = 0;
  const QVector<HKLFamily::Pointer> families = p->getHKLFamilies();
  HKLFamily_t hkl;
  for(const HKLFamily::Pointer& family : families)
  {
    family->copyToStruct(&hkl);

    memtype = H5Tcreate(H5T_COMPOUND, sizeof(HKLFamily_t));
    status = H5Tinsert(memtype, "H", HOFFSET(HKLFamily_t, h), H5T_NATIVE_INT);
    status = H5Tinsert(memtype, "K", HOFFSET(HKLFamily_t, k), H5T_NATIVE_INT);
    status = H5Tinsert(memtype, "L", HOFFSET(HKLFamily_t, l), H5T_NATIVE_INT);
    status = H5Tinsert(memtype, "Solution 1", HOFFSET(HKLFamily_t, s1), H5T_NATIVE_CHAR);
    status = H5Tinsert(memtype, "Diffraction Intensity", HOFFSET(HKLFamily_t, diffractionIntensity), H5T_NATIVE_FLOAT);
    status = H5Tinsert(memtype, "Solution 2", HOFFSET(HKLFamily_t, s2), H5T_NATIVE_CHAR);

    /*
     * Create dataspace.  Setting maximum size to nullptr sets the maximum
     * size to be the current size.
     */
    space = H5Screate_simple(1, dims, nullptr);

    /*
     * Create the dataset and write the compound data to it.
     */
    QString indexStr = QString::number(index++);
    dset = H5Dcreate(hklGid, indexStr.toLatin1().data(), memtype, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    status = H5Dwrite(dset, memtype, H5S_ALL, H5S_ALL, H5P_DEFAULT, (void*)(&hkl));

    /*
     * Close and release resources.
     */
    status = H5Tclose(memtype);
    status = H5Sclose(space);
    status = H5Dclose(dset);
  }
  return status;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5AngImporter::setFileVersion(uint32_t version)
{
  m_FileVersion = version;
}

// -----------------------------------------------------------------------------
H5AngImporter::Pointer H5AngImporter::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
QString H5AngImporter::getNameOfClass() const
{
  return QString("H5AngImporter");
}

// -----------------------------------------------------------------------------
QString H5AngImporter::ClassName()
{
  return QString("H5AngImporter");
}
