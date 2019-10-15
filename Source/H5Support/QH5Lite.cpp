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

#include "QH5Lite.h"

#include <cstring>
#include <string>

#include <QtCore/QDebug>

#if defined(H5Support_NAMESPACE)
using namespace H5Support;
#endif

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void QH5Lite::disableErrorHandlers()
{
  H5SUPPORT_MUTEX_LOCK()

  HDF_ERROR_HANDLER_OFF;
}

// -----------------------------------------------------------------------------
//  Opens an ID for HDF5 operations
// -----------------------------------------------------------------------------
hid_t QH5Lite::openId(hid_t locationID, const QString& objectName, H5O_type_t objectType)
{
  return H5Lite::openId(locationID, objectName.toStdString(), objectType);
}

// -----------------------------------------------------------------------------
//  Closes the given ID
// -----------------------------------------------------------------------------
herr_t QH5Lite::closeId(hid_t objectID, int32_t objectType)
{
  return H5Lite::closeId(objectID, objectType);
}

// -----------------------------------------------------------------------------
//  Finds an Attribute given an object to look in
// -----------------------------------------------------------------------------
herr_t QH5Lite::findAttribute(hid_t locationID, const QString& attributeName)
{
  return H5Lite::findAttribute(locationID, attributeName.toStdString());
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool QH5Lite::datasetExists(hid_t locationID, const QString& name)
{
  return H5Lite::datasetExists(locationID, name.toStdString());
}

// -----------------------------------------------------------------------------
//  We assume a null terminated string
// -----------------------------------------------------------------------------
herr_t QH5Lite::writeStringDataset(hid_t locationID, const QString& datasetName, size_t size, const char* data)
{
  return H5Lite::writeStringDataset(locationID, datasetName.toStdString(), size, data);
}

// -----------------------------------------------------------------------------
//  Writes a string to a HDF5 dataset
// -----------------------------------------------------------------------------
herr_t QH5Lite::writeStringDataset(hid_t locationID, const QString& datasetName, const QString& data)
{
  return H5Lite::writeStringDataset(locationID, datasetName.toStdString(), data.toStdString());
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t QH5Lite::writeVectorOfStringsDataset(hid_t locationID, const QString& datasetName, const QVector<QString>& data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t dataspaceID = -1;
  hid_t memSpace = -1;
  hid_t datatype = -1;
  hid_t datasetID = -1;
  herr_t error = -1;
  herr_t returnError = 0;

  std::array<hsize_t, 1> dims = {static_cast<hsize_t>(data.size())};
  if((dataspaceID = H5Screate_simple(static_cast<int>(dims.size()), dims.data(), nullptr)) >= 0)
  {
    dims[0] = 1;

    if((memSpace = H5Screate_simple(static_cast<int>(dims.size()), dims.data(), nullptr)) >= 0)
    {

      datatype = H5Tcopy(H5T_C_S1);
      H5Tset_size(datatype, H5T_VARIABLE);

      if((datasetID = H5Dcreate(locationID, datasetName.toLocal8Bit().constData(), datatype, dataspaceID, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT)) >= 0)
      {
        // Select the "memory" to be written out - just 1 record.
        hsize_t offset[] = {0};
        hsize_t count[] = {1};
        H5Sselect_hyperslab(memSpace, H5S_SELECT_SET, offset, nullptr, count, nullptr);
        hsize_t pos = 0;
        for(const auto& element : data)
        {
          // Select the file position, 1 record at position 'pos'
          hsize_t count[] = {1};
          hsize_t offset[] = {pos};
          pos++;
          H5Sselect_hyperslab(dataspaceID, H5S_SELECT_SET, offset, nullptr, count, nullptr);
          std::string elementStr = element.toStdString();
          const char* strPtr = elementStr.c_str();
          error = H5Dwrite(datasetID, datatype, memSpace, dataspaceID, H5P_DEFAULT, &strPtr);
          if(error < 0)
          {
            qDebug() << "Error Writing String Data: " __FILE__ << "(" << __LINE__ << ")";
            returnError = error;
          }
        }
        QCloseH5D(datasetID, error, returnError, datasetName);
      }
      H5Tclose(datatype);
      QCloseH5S(memSpace, error, returnError);
    }

    QCloseH5S(dataspaceID, error, returnError);
  }
  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t QH5Lite::writeStringAttributes(hid_t locationID, const QString& objectName, const QMap<QString, QString>& attributes)
{
  herr_t error = 0;
  QMapIterator<QString, QString> iter(attributes);
  while(iter.hasNext())
  {
    iter.next();
    error = H5Lite::writeStringAttribute(locationID, objectName.toStdString(), iter.key().toStdString(), iter.value().toStdString());
    if(error < 0)
    {
      return error;
    }
  }
  return error;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
hsize_t QH5Lite::getNumberOfElements(hid_t locationID, const QString& datasetName)
{
  return H5Lite::getNumberOfElements(locationID, datasetName.toStdString());
}

// -----------------------------------------------------------------------------
//  Writes a string to an HDF5 Attribute
// -----------------------------------------------------------------------------
herr_t QH5Lite::writeStringAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, hsize_t size, const char* data)
{
  return H5Lite::writeStringAttribute(locationID, objectName.toStdString(), attributeName.toStdString(), size, data);
}

// -----------------------------------------------------------------------------
//  Writes a string to an HDF5 Attribute
// -----------------------------------------------------------------------------
herr_t QH5Lite::writeStringAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, const QString& data)
{
  return H5Lite::writeStringAttribute(locationID, objectName.toStdString(), attributeName.toStdString(), data.size() + 1, data.toLatin1().data());
}

// -----------------------------------------------------------------------------
//  Reads a String dataset into a QString
// -----------------------------------------------------------------------------
herr_t QH5Lite::readStringDataset(hid_t locationID, const QString& datasetName, QString& data)
{
  std::string readValue;
  herr_t error = H5Lite::readStringDataset(locationID, datasetName.toStdString(), readValue);
  data = QString::fromStdString(readValue);
  return error;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t QH5Lite::readStringDataset(hid_t locationID, const QString& datasetName, char* data)
{
  return H5Lite::readStringDataset(locationID, datasetName.toStdString(), data);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t QH5Lite::readVectorOfStringDataset(hid_t locationID, const QString& datasetName, QVector<QString>& data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID; // dataset id
  hid_t typeID;    // type id
  herr_t error = 0;
  herr_t returnError = 0;

  datasetID = H5Dopen(locationID, datasetName.toLocal8Bit().constData(), H5P_DEFAULT);
  if(datasetID < 0)
  {
    qDebug() << "QH5Lite.cpp::readVectorOfStringDataset(" << __LINE__ << ") Error opening Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")";
    return -1;
  }
  /*
   * Get the datatype.
   */
  typeID = H5Dget_type(datasetID);
  if(typeID >= 0)
  {
    hsize_t dims[1] = {0};
    /*
     * Get dataspace and allocate memory for read buffer.
     */
    hid_t dataspaceID = H5Dget_space(datasetID);
    int ndims = H5Sget_simple_extent_dims(dataspaceID, dims, nullptr);
    if(ndims != 1)
    {
      CloseH5S(dataspaceID, error, returnError);
      CloseH5T(typeID, error, returnError);
      qDebug() << "QH5Lite.cpp::readVectorOfStringDataset(" << __LINE__ << ") Number of dims should be 1 but it was " << ndims << ". Returning early. Is your data file correct?";
      return -2;
    }

    std::vector<char*> rData(dims[0], nullptr);

    /*
     * Create the memory datatype.
     */
    hid_t memtype = H5Tcopy(H5T_C_S1);
    herr_t status = H5Tset_size(memtype, H5T_VARIABLE);

    /*
     * Read the data.
     */
    status = H5Dread(datasetID, memtype, H5S_ALL, H5S_ALL, H5P_DEFAULT, rData.data());
    if(status < 0)
    {
      status = H5Dvlen_reclaim(memtype, dataspaceID, H5P_DEFAULT, rData.data());
      CloseH5S(dataspaceID, error, returnError);
      CloseH5T(typeID, error, returnError);
      CloseH5T(memtype, error, returnError);
      qDebug() << "QH5Lite.cpp::readVectorOfStringDataset(" << __LINE__ << ") Error reading Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")";
      return -3;
    }
    data.resize(dims[0]);
    /*
     * copy the data into the vector of strings
     */
    for(int i = 0; i < dims[0]; i++)
    {
      // printf("%s[%d]: %s\n", "VlenStrings", i, rData[i].p);
      data[i] = QString::fromLatin1(rData[i]);
    }
    /*
     * Close and release resources.  Note that H5Dvlen_reclaim works
     * for variable-length strings as well as variable-length arrays.
     * Also note that we must still free the array of pointers stored
     * in rData, as H5Tvlen_reclaim only frees the data these point to.
     */
    status = H5Dvlen_reclaim(memtype, dataspaceID, H5P_DEFAULT, rData.data());
    QCloseH5S(dataspaceID, error, returnError);
    QCloseH5T(typeID, error, returnError);
    QCloseH5T(memtype, error, returnError);
  }

  QCloseH5D(datasetID, error, returnError, datasetName);

  return returnError;
}

// -----------------------------------------------------------------------------
//  Reads a string Attribute from the HDF file
// -----------------------------------------------------------------------------
herr_t QH5Lite::readStringAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, QString& data)
{
  std::string sValue;
  herr_t error = H5Lite::readStringAttribute(locationID, objectName.toStdString(), attributeName.toStdString(), sValue);
  data = QString::fromStdString(sValue);
  return error;
}

// -----------------------------------------------------------------------------
//  Reads a string Attribute from the HDF file
// -----------------------------------------------------------------------------
herr_t QH5Lite::readStringAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, char* data)
{
  return H5Lite::readStringAttribute(locationID, objectName.toStdString(), attributeName.toStdString(), data);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t QH5Lite::getDatasetNDims(hid_t locationID, const QString& datasetName, hid_t& rank)
{
  return H5Lite::getDatasetNDims(locationID, datasetName.toStdString(), rank);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t QH5Lite::getAttributeNDims(hid_t locationID, const QString& objectName, const QString& attributeName, hid_t& rank)
{
  return H5Lite::getAttributeNDims(locationID, objectName.toStdString(), attributeName.toStdString(), rank);
}

// -----------------------------------------------------------------------------
//  Returns the type of data stored in the dataset. You MUST use H5Tclose(typeID)
//  on the returned value or resource leaks will occur.
// -----------------------------------------------------------------------------
hid_t QH5Lite::getDatasetType(hid_t locationID, const QString& datasetName)
{
  return H5Lite::getDatasetType(locationID, datasetName.toStdString());
}

// -----------------------------------------------------------------------------
//  Get the dataset information
// -----------------------------------------------------------------------------
herr_t QH5Lite::getDatasetInfo(hid_t locationID, const QString& datasetName, QVector<hsize_t>& dims, H5T_class_t& classType, size_t& sizeType)
{
  // Since this is a wrapper we need to pass a std::vector() then copy the values from that into our 'dims' argument
  std::vector<hsize_t> rDims;
  herr_t error = H5Lite::getDatasetInfo(locationID, datasetName.toStdString(), rDims, classType, sizeType);
  dims.resize(static_cast<qint32>(rDims.size()));
  for(std::vector<hsize_t>::size_type i = 0; i < rDims.size(); ++i)
  {
    dims[static_cast<qint32>(i)] = rDims[i];
  }
  return error;
}

// -----------------------------------------------------------------------------
//  You must close the attributeType argument or resource leaks will occur. Use
//  H5Tclose(typeID); after your call to this method if you do not need the id for
//   anything.
// -----------------------------------------------------------------------------
herr_t QH5Lite::getAttributeInfo(hid_t locationID, const QString& objectName, const QString& attributeName, QVector<hsize_t>& dims, H5T_class_t& type_class, size_t& type_size, hid_t& typeID)
{
  std::vector<hsize_t> rDims = dims.toStdVector();
  herr_t error = H5Lite::getAttributeInfo(locationID, objectName.toStdString(), attributeName.toStdString(), rDims, type_class, type_size, typeID);
  dims.resize(static_cast<qint32>(rDims.size()));
  for(std::vector<hsize_t>::size_type i = 0; i < rDims.size(); ++i)
  {
    dims[static_cast<qint32>(i)] = rDims[i];
  }
  return error;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
hid_t QH5Lite::HDFTypeFromString(const QString& value)
{
  return H5Lite::HDFTypeFromString(value.toStdString());
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString QH5Lite::StringForHDFType(hid_t type)
{
  return QString::fromStdString(H5Lite::StringForHDFType(type));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QVector<hsize_t> QH5Lite::guessChunkSize(const QVector<hsize_t>& dims, size_t typeSize)
{
  return QVector<hsize_t>::fromStdVector(H5Lite::guessChunkSize(dims.size(), dims.data(), typeSize));
}
