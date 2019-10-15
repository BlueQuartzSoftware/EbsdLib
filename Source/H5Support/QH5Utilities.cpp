/* ============================================================================
 * Copyright (c) 2009-2019 BlueQuartz Software, LLC
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
 *    United States Air Force Prime Contract FA8650-15-D-5231
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "QH5Utilities.h"

#include <QtCore/QDebug>
#include <QtCore/QFileInfo>

#include "H5Support/H5Utilities.h"

#if defined(H5Support_NAMESPACE)
using namespace H5Support;
#endif

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
hid_t QH5Utilities::createFile(const QString& filename)
{
  return H5Utilities::createFile(filename.toStdString());
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
hid_t QH5Utilities::openFile(const QString& filename, bool readOnly)
{
  return H5Utilities::openFile(filename.toStdString(), readOnly);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t QH5Utilities::closeFile(hid_t& fileId)
{
  return H5Utilities::closeFile(fileId);
}

// -----------------------------------------------------------------------------
//  Returns the full path to the object referred to by the
// -----------------------------------------------------------------------------
QString QH5Utilities::getObjectPath(hid_t locationID, bool trim)
{
  return QString::fromStdString(H5Utilities::getObjectPath(locationID, trim));
}

// -----------------------------------------------------------------------------
// @brief Retrieves the HDF object type for objectName at locationID and stores
//    it in the parameter obj_type passed in.
// -----------------------------------------------------------------------------
herr_t QH5Utilities::getObjectType(hid_t objectID, const QString& objectName, int32_t& objectType)
{
  return H5Utilities::getObjectType(objectID, objectName.toStdString(), objectType);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString QH5Utilities::getParentPath(hid_t objectID)
{
  return QString::fromStdString(H5Utilities::getParentPath(objectID));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString QH5Utilities::getParentPath(const QString& objectPath)
{
  return QString::fromStdString(H5Utilities::getParentPath(objectPath.toStdString()));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString QH5Utilities::getObjectNameFromPath(const QString& objectPath)
{
  return QString::fromStdString(H5Utilities::getObjectNameFromPath(objectPath.toStdString()));
}

// Opens and returns the HDF object (since the HDF api requires
//  different open and close methods for different types of objects
hid_t QH5Utilities::openHDF5Object(hid_t locationID, const QString& objectName)
{
  return H5Utilities::openHDF5Object(locationID, objectName.toStdString());
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t QH5Utilities::closeHDF5Object(hid_t objectID)
{
  return H5Utilities::closeHDF5Object(objectID);
}

//--------------------------------------------------------------------//
// HDF Group Methods
//--------------------------------------------------------------------//
herr_t QH5Utilities::getGroupObjects(hid_t locationID, H5Utilities::CustomHDFDataTypes typeFilter, QList<QString>& names)
{
  std::list<std::string> childNames;
  herr_t error = H5Utilities::getGroupObjects(locationID, static_cast<int32_t>(typeFilter), childNames);

  names.clear();
  for(const auto& childName : childNames)
  {
    names.push_back(QString::fromStdString(childName));
  }

  return error;
}

// -----------------------------------------------------------------------------
// HDF Creation/Modification Methods
// -----------------------------------------------------------------------------
hid_t QH5Utilities::createGroup(hid_t locationID, const QString& group)
{
  return H5Utilities::createGroup(locationID, group.toStdString());
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int32_t QH5Utilities::createGroupsForDataset(const QString& datasetPath, hid_t parent)
{
  return H5Utilities::createGroupsForDataset(datasetPath.toStdString(), parent);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int32_t QH5Utilities::createGroupsFromPath(const QString& pathToCheck, hid_t parent)
{
  return H5Utilities::createGroupsFromPath(pathToCheck.toStdString(), parent);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString QH5Utilities::extractObjectName(const QString& path)
{
  return QString::fromStdString(H5Utilities::extractObjectName(path.toStdString()));
}

//--------------------------------------------------------------------//
// HDF Attribute Methods
//--------------------------------------------------------------------//
bool QH5Utilities::probeForAttribute(hid_t locationID, const QString& objectName, const QString& attributeName)
{
  return H5Utilities::probeForAttribute(locationID, objectName.toStdString(), attributeName.toStdString());
}

//--------------------------------------------------------------------//
// Returns a QList of all attribute names attached to the object
//  referred to by objectID
//--------------------------------------------------------------------//
herr_t QH5Utilities::getAllAttributeNames(hid_t objectID, QList<QString>& names)
{
  names.clear();
  std::list<std::string> attributeNames;
  herr_t err = H5Utilities::getAllAttributeNames(objectID, attributeNames);
  for(const auto& attributeName : attributeNames)
  {
    names.push_back(QString::fromStdString(attributeName));
  }
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t QH5Utilities::getAllAttributeNames(hid_t locationID, const QString& objectName, QList<QString>& names)
{
  names.clear();
  std::list<std::string> attributeNames;
  herr_t err = H5Utilities::getAllAttributeNames(locationID, objectName.toStdString(), attributeNames);
  for(const auto& attributeName : attributeNames)
  {
    names.push_back(QString::fromStdString(attributeName));
  }
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString QH5Utilities::HDFClassTypeAsStr(hid_t classType)
{
  return QString::fromStdString(H5Utilities::HDFClassTypeAsStr(classType));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void QH5Utilities::printHDFClassType(H5T_class_t classType)
{
  std::string hType = H5Utilities::HDFClassTypeAsStr(classType);
  qDebug() << QString::fromStdString(hType);
}

// -----------------------------------------------------------------------------
//  Returns a QString that is the name of the object at the given index
// -----------------------------------------------------------------------------
herr_t QH5Utilities::objectNameAtIndex(hid_t fileId, int32_t index, QString& name)
{
  std::string sName;
  herr_t error = H5Utilities::objectNameAtIndex(fileId, index, sName);
  name = QString::fromStdString(sName);
  return error;
}

// -----------------------------------------------------------------------------
// Checks the given name object to see what type of HDF5 object it is.
// -----------------------------------------------------------------------------
bool QH5Utilities::isGroup(hid_t nodeID, const QString& objectName)
{
  return H5Utilities::isGroup(nodeID, objectName.toStdString());
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString QH5Utilities::fileNameFromFileId(hid_t fileId)
{
  H5SUPPORT_MUTEX_LOCK()

  // Get the name of the .dream3d file that we are writing to:
  ssize_t nameSize = H5Fget_name(fileId, nullptr, 0) + 1;
  QByteArray nameBuffer(nameSize, 0);
  nameSize = H5Fget_name(fileId, nameBuffer.data(), nameSize);

  QString hdfFileName(nameBuffer);
  QFileInfo fileInfo(hdfFileName);
  hdfFileName = fileInfo.fileName();
  return hdfFileName;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString QH5Utilities::absoluteFilePathFromFileId(hid_t fileId)
{
  H5SUPPORT_MUTEX_LOCK()

  // Get the name of the .dream3d file that we are writing to:
  ssize_t nameSize = H5Fget_name(fileId, nullptr, 0) + 1;
  QByteArray nameBuffer(nameSize, 0);
  nameSize = H5Fget_name(fileId, nameBuffer.data(), nameSize);

  QString hdfFileName(nameBuffer);
  QFileInfo fileInfo(hdfFileName);
  return fileInfo.absoluteFilePath();
}
