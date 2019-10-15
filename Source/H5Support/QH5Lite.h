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

#pragma once

#include <typeinfo>

#include <H5Tpublic.h>
#include <hdf5.h>

#include "H5Support/H5Lite.h"
#include "H5Support/H5Support.h"

#ifndef H5Support_USE_QT
#pragma message("THIS FILE SHOULD NOT BE INCLUDED UNLESS THE H5Support_USE_QT is also defined")
#endif

#include <QtCore/QString>
#include <QtCore/QVector>

#if defined(H5Support_NAMESPACE)
namespace H5Support
{
#endif

/**
 * @brief Namespace to bring together some high level methods to read/write data to HDF5 files.
 * @class QH5Lite
 * @author Mike Jackson
 * @date April 2007
 * @version $Revision: 1.3 $
 */
namespace QH5Lite
{
/**
 * @brief Turns off the global error handler/reporting objects. Note that once
 * they are turned off using this method they CAN NOT be turned back on. If you
 * would like to turn them off for a piece of code then surround your code with
 * the HDF_ERROR_HANDLER_OFF and HDF_ERROR_HANDLER_ON macros defined in
 * QH5Lite.h
 */
H5Support_EXPORT void disableErrorHandlers();

/**
 * @brief Opens an object for HDF5 operations
 * @param locationID The parent object that holds the true object we want to open
 * @param objectName The string name of the object
 * @param objectType The HDF5_TYPE of object
 * @return Standard HDF5 Error Conditions
 */
H5Support_EXPORT hid_t openId(hid_t locationID, const QString& objectName, H5O_type_t objectType);

/**
 * @brief Opens an HDF5 Object
 * @param objectID The Object id
 * @param objectType Basic Object Type
 * @return Standard HDF5 Error Conditions
 */
H5Support_EXPORT herr_t closeId(hid_t objectID, int32_t objectType);

/**
 * @brief Given one of the HDF Types as a string, this will return the HDF Type
 * as an hid_t value.
 * @param value The HDF_Type as a string
 * @return the hid_t value for the given type. -1 if the string does not match a type.
 */
H5Support_EXPORT hid_t HDFTypeFromString(const QString& value);

/**
 * @brief Returns a string version of the HDF Type
 * @param type The HDF5 Type to query
 * @return
 */
H5Support_EXPORT QString StringForHDFType(hid_t type);

/**
 * @brief Returns the HDF Type for a given primitive value.
 * @param value A value to use. Can be anything. Just used to get the type info
 * from
 * @return A QString representing the HDF5 Type
 */
template <typename T> QString HDFTypeForPrimitiveAsStr(T value)
{
  return QString::fromStdString(H5Lite::HDFTypeForPrimitiveAsStr(value));
}

/**
 * @brief Returns the HDF Type for a given primitive value.
 * @param value A value to use. Can be anything. Just used to get the type info
 * from
 * @return The HDF5 native type for the value
 */
template <typename T> hid_t HDFTypeForPrimitive(T value)
{
  return H5Lite::HDFTypeForPrimitive(value);
}

/**
 * @brief Inquires if an attribute named attr_name exists attached to the object locationID.
 * @param locationID The location to search
 * @param attributeName The attribute to search for
 * @return Standard HDF5 Error condition
 */
H5Support_EXPORT herr_t findAttribute(hid_t locationID, const QString& attributeName);

/**
 * @brief Finds a Data set given a data set name
 * @param locationID The location to search
 * @param name The dataset to search for
 * @return Standard HDF5 Error condition. Negative=DataSet
 */
H5Support_EXPORT bool datasetExists(hid_t locationID, const QString& name);

/**
 * @brief Creates a Dataset with the given name at the location defined by locationID
 *
 *
 * @param locationID The Parent location to store the data
 * @param datasetName The name of the dataset
 * @param dims The dimensions of the dataset
 * @param data The data to write to the file
 * @return Standard HDF5 error conditions
 *
 * The dimensions of the data sets are usually passed as both a "rank" and
 * dimensions array. By using a QVector<hsize_t> that stores the values of
 * each of the dimensions we can reduce the number of arguments to this method as
 * the value of the "rank" simply becomes dims.length(). So to create a Dims variable
 * for a 3D data space of size(x,y,z) = {10,20,30} I would use the following code:
 * <code>
 * QVector<hsize_t> dims;
 * dims.push_back(10);
 * dims.push_back(20);
 * dims.push_back(30);
 * </code>
 *
 * Also when passing data BE SURE that the type of data and the data type match.
 * For example if I create some data in a QVector<UInt8Type> you would need to
 * pass H5T_NATIVE_UINT8 as the dataType.
 *
 * Also note that QVector is 32 bit limited in that the most data that it can hold is 2^31 elements. If you
 * are trying to write a data set that has more than 2^31 elements then use the H5Lite::writeVectorDataset()
 * instead which takes a std::vector() and is probably more suited for large data.
 */
template <typename T> herr_t writeVectorDataset(hid_t locationID, const QString& datasetName, const QVector<hsize_t>& dims, const QVector<T>& data)
{
  return H5Lite::writePointerDataset(locationID, datasetName.toStdString(), dims.size(), dims.data(), data.data());
}

/**
 * @brief Returns a guess for the vector of chunk dimensions based on the input parameters.
 * @param dims The vector dimensions of the dataset
 * @param typeSize The size of the data type for the dataset
 * @return The vector of chunk dimensions guess
 */
H5Support_EXPORT QVector<hsize_t> guessChunkSize(const QVector<hsize_t>& dims, size_t typeSize);

#ifdef H5_HAVE_FILTER_DEFLATE
/**
 * @brief Creates a Dataset with the given name at the location defined by locationID with the given compression
 *
 * @param locationID The Parent location to store the data
 * @param datasetName The name of the dataset
 * @param rank The number of dimensions
 * @param dims The dimensions of the dataset
 * @param data The data to write to the file
 * @param cRank The number of dimensions for cDims
 * @param cDims The chunk dimensions
 * @param compressionLevel The compression level (0-9)
 * @return Standard HDF5 error conditions
 */
template <typename T>
herr_t writePointerDatasetCompressed(hid_t locationID, const QString& datasetName, int32_t rank, const hsize_t* dims, const T* data, int32_t cRank, const hsize_t* cDims, int32_t compressionLevel)
{
  return H5Lite::writePointerDatasetCompressed(locationID, datasetName.toStdString(), rank, dims, data, cRank, cDims, compressionLevel);
}

/**
 * @brief Creates a Dataset with the given name at the location defined by locationID with the given compression
 *
 * @param locationID The Parent location to store the data
 * @param datasetName The name of the dataset
 * @param dims The dimensions of the dataset
 * @param data The data to write to the file
 * @param cDims The chunk dimensions
 * @param compressionLevel The compression level (0-9)
 * @return Standard HDF5 error conditions
 */
template <typename T>
herr_t writeVectorDatasetCompressed(hid_t locationID, const QString& datasetName, const QVector<hsize_t>& dims, const QVector<T>& data, const QVector<hsize_t>& cDims, int32_t compressionLevel)
{
  return H5Lite::writePointerDatasetCompressed(locationID, datasetName.toStdString(), dims.size(), dims.data(), data.data(), cDims.size(), cDims.data(), compressionLevel);
}
#endif

/**
 * @brief Writes the data of a pointer to an HDF5 file
 * @param locationID The hdf5 object id of the parent
 * @param datasetName The name of the dataset to write to. This can be a name of Path
 * @param rank The number of dimensions
 * @param dims The sizes of each dimension
 * @param data The data to be written.
 * @return Standard hdf5 error condition.
 */
template <typename T> herr_t writePointerDataset(hid_t locationID, const QString& datasetName, int32_t rank, const hsize_t* dims, const T* data)
{
  return H5Lite::writePointerDataset(locationID, datasetName.toStdString(), rank, dims, data);
}

/**
 * @brief Replaces the given dataset with the data of a pointer to an HDF5 file. Creates the dataset if it does not exist.
 * @param locationID The hdf5 object id of the parent
 * @param datasetName The name of the dataset to write to. This can be a name of Path
 * @param rank The number of dimensions
 * @param dims The sizes of each dimension
 * @param data The data to be written.
 * @return Standard hdf5 error condition.
 */
template <typename T> herr_t replacePointerDataset(hid_t locationID, const QString& datasetName, int32_t rank, const hsize_t* dims, const T* data)
{
  return H5Lite::replacePointerDataset(locationID, datasetName.toStdString(), rank, dims, data);
}

/**
 * @brief Creates a Dataset with the given name at the location defined by locationID.
 * This version of writeDataset should be used with a single scalar value. If you
 * need to write an array of values, use the form that takes an QVector<>
 *
 * @param locationID The Parent location to store the data
 * @param datasetName The name of the dataset
 * @param value The value to write to the HDF5 dataset
 * @return Standard HDF5 error conditions
 */
template <typename T> herr_t writeScalarDataset(hid_t locationID, const QString& datasetName, T& value)
{
  return H5Lite::writeScalarDataset(locationID, datasetName.toStdString(), value);
}

/**
 * @brief Writes a QString as a HDF Dataset.
 * @param locationID The Parent location to write the dataset
 * @param datasetName The Name to use for the dataset
 * @param data The actual data to write as a null terminated string
 * @return Standard HDF5 error conditions
 */
H5Support_EXPORT herr_t writeStringDataset(hid_t locationID, const QString& datasetName, const QString& data);

/**
 * @brief Writes a null terminated 'C String' to an HDF Dataset.
 * @param locationID The Parent location to write the dataset
 * @param datasetName The Name to use for the dataset
 * @param data const char pointer to write as a null terminated string
 * @param size The number of characters in the string
 * @return Standard HDF5 error conditions
 */
H5Support_EXPORT herr_t writeStringDataset(hid_t locationID, const QString& datasetName, size_t size, const char* data);

/**
 * @brief
 * @param locationID
 * @param datasetName
 * @param size
 * @param data
 * @return
 */
H5Support_EXPORT herr_t writeVectorOfStringsDataset(hid_t locationID, const QString& datasetName, const QVector<QString>& data);
/**
 * @brief Writes an Attribute to an HDF5 Object
 * @param locationID The Parent Location of the HDFobject that is getting the attribute
 * @param objectName The Name of Object to write the attribute into.
 * @param attributeName The Name of the Attribute
 * @param rank The number of dimensions in the attribute data
 * @param dims The Dimensions of the attribute data
 * @param data The Attribute Data to write as a pointer
 * @return Standard HDF Error Condition
 */
template <typename T> herr_t writePointerAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, int32_t rank, const hsize_t* dims, const T* data)
{
  return H5Lite::writePointerAttribute(locationID, objectName.toStdString(), attributeName.toStdString(), rank, dims, data);
}

/**
 * @brief Writes an Attribute to an HDF5 Object
 * @param locationID The Parent Location of the HDFobject that is getting the attribute
 * @param objectName The Name of Object to write the attribute into.
 * @param attributeName The Name of the Attribute
 * @param dims The Dimensions of the data set
 * @param data The Attribute Data to write
 * @return Standard HDF Error Condition
 *
 */
template <typename T> herr_t writeVectorAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, const QVector<hsize_t>& dims, const QVector<T>& data)
{
  return H5Lite::writePointerAttribute(locationID, objectName.toStdString(), attributeName.toStdString(), dims.size(), dims.data(), data.data());
}

/**
 * @brief Writes a string as a null terminated attribute.
 * @param locationID The location to look for objectName
 * @param objectName The Object to write the attribute to
 * @param attributeName The name of the Attribute
 * @param data The string to write as the attribute
 * @return Standard HDF error conditions
 */
H5Support_EXPORT herr_t writeStringAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, const QString& data);
/**
 * @brief Writes a null terminated string as an attribute
 * @param locationID The location to look for objectName
 * @param objectName The Object to write the attribute to
 * @param attributeName The name of the Attribute
 * @param size The number of characters  in the string
 * @param data pointer to a const char array
 * @return Standard HDF error conditions
 */
H5Support_EXPORT herr_t writeStringAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, hsize_t size, const char* data);

/**
 * @brief Writes attributes that all have a data type of STRING. The first value
 * in each set is the key, the second is the actual value of the attribute.
 * @param locationID The location to look for objectName
 * @param objectName The Object to write the attribute to
 * @param attributes The attributes to be written where the first value is the name
 * of the attribute, and the second is the actual value of the attribute.
 * @return Standard HDF error condition
 */
H5Support_EXPORT herr_t writeStringAttributes(hid_t locationID, const QString& objectName, const QMap<QString, QString>& attributes);

/**
 * @brief Returns the total number of elements in the supplied dataset
 * @param locationID The parent location that contains the dataset to read
 * @param datasetName The name of the dataset to read
 * The best idea is to just allocate the vector but not to size it. The method
 * will size it for you.
 * @return Number of elements in dataset
 */
H5Support_EXPORT hsize_t getNumberOfElements(hid_t locationID, const QString& datasetName);

/**
 * @brief Writes an attribute to the given object. This method is designed with
 * a Template parameter that represents a primitive value. If you need to write
 * an array, please use the other over loaded method that takes a vector.
 * @param locationID The location to look for objectName
 * @param objectName The Object to write the attribute to
 * @param attributeName The  name of the attribute
 * @param data The data to be written as the attribute
 * @return Standard HDF error condition
 */
template <typename T> herr_t writeScalarAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, T data)
{
  return H5Lite::writeScalarAttribute(locationID, objectName.toStdString(), attributeName.toStdString(), data);
}

/**
 * @brief Reads data from the HDF5 File into a preallocated array.
 * @param locationID The parent location that contains the dataset to read
 * @param datasetName The name of the dataset to read
 * @param data A Pointer to the PreAllocated Array of Data
 * @return Standard HDF error condition
 */
template <typename T> herr_t readPointerDataset(hid_t locationID, const QString& datasetName, T* data)
{
  return H5Lite::readPointerDataset(locationID, datasetName.toStdString(), data);
}

/**
 * @brief Get the information about a dataset.
 *
 * @param locationID The parent location of the Dataset
 * @param datasetName The name of the dataset
 * @param dims A QVector that will hold the sizes of the dimensions
 * @param type_class The HDF5 class type
 * @param type_size THe HDF5 size of the data
 * @return Negative value is Failure. Zero or Positive is success;
 */
H5Support_EXPORT herr_t getDatasetInfo(hid_t locationID, const QString& datasetName, QVector<hsize_t>& dims, H5T_class_t& classType, size_t& sizeType);

/**
 * @brief Reads data from the HDF5 File into an QVector<T> object. If the dataset
 * is very large this can be an expensive method to use. It is here for convenience
 * using STL with hdf5.
 * @param locationID The parent location that contains the dataset to read
 * @param datasetName The name of the dataset to read
 * @param data A QVector<T>. Note the vector WILL be resized to fit the data.
 * The best idea is to just allocate the vector but not to size it. The method
 * will size it for you.
 * @return Standard HDF error condition
 */
template <typename T> herr_t readVectorDataset(hid_t locationID, const QString& datasetName, QVector<T>& data)
{
  std::string datasetNameStr = datasetName.toStdString();
  QVector<hsize_t> dims;
  H5T_class_t classType;
  size_t sizeType;
  herr_t error = getDatasetInfo(locationID, datasetName, dims, classType, sizeType) < 0;
  if(error < 0)
  {
    return error;
  }
  hsize_t numElements = std::accumulate(dims.cbegin(), dims.cend(), static_cast<hsize_t>(1), std::multiplies<hsize_t>());
  data.resize(static_cast<int>(numElements));

  error = H5Lite::readPointerDataset(locationID, datasetNameStr, data.data());
  return error;
}

/**
 * @brief Reads a dataset that consists of a single scalar value
 * @param locationID The HDF5 file or group id
 * @param datasetName The name or path to the dataset to read
 * @param data The variable to store the data into
 * @return HDF error condition.
 */
template <typename T> herr_t readScalarDataset(hid_t locationID, const QString& datasetName, T& data)
{
  std::string datasetNameStr = datasetName.toStdString();
  herr_t error = H5Lite::readScalarDataset(locationID, datasetNameStr, data);
  return error;
}

/**
 * @brief Reads a string dataset into the supplied string. Any data currently in the 'data' variable
 * is cleared first before the new data is read into the string.
 * @param locationID The parent group that holds the data object to read
 * @param datasetName The name of the dataset.
 * @param data The QString to hold the data
 * @return Standard HDF error condition
 */
H5Support_EXPORT herr_t readStringDataset(hid_t locationID, const QString& datasetName, QString& data);

/**
 * @brief
 * @param locationID
 * @param datasetName
 * @param data
 * @return
 */
H5Support_EXPORT herr_t readVectorOfStringDataset(hid_t locationID, const QString& datasetName, QVector<QString>& data);
/**
 * @brief reads a null terminated string dataset into the supplied buffer. The buffer
 * should be already preallocated.
 * @param locationID The parent group that holds the data object to read
 * @param datasetName The name of the dataset.
 * @param data pointer to the buffer
 * @return Standard HDF error condition
 */
H5Support_EXPORT herr_t readStringDataset(hid_t locationID, const QString& datasetName, char* data);

/**
 * @brief Returns the information about an attribute.
 * You must close the attributeType argument or resource leaks will occur. Use
 *  H5Tclose(attr_type); after your call to this method if you do not need the id for
 *   anything.
 * @param locationID The parent location of the Dataset
 * @param objectName The name of the dataset
 * @param attr_name The name of the attribute
 * @param dims A QVector that will hold the sizes of the dimensions
 * @param type_class The HDF5 class type
 * @param type_size THe HDF5 size of the data
 * @param attr_type The Attribute ID - which needs to be closed after you are finished with the data
 * @return
 */
H5Support_EXPORT herr_t getAttributeInfo(hid_t locationID, const QString& objectName, const QString& attributeName, QVector<hsize_t>& dims, H5T_class_t& type_class, size_t& type_size, hid_t& tid);

/**
 * @brief Reads an Attribute from an HDF5 Object.
 *
 * Use this method if you already know the datatype of the attribute. If you do
 * not know this already then use another form of this method.
 *
 * @param locationID The Parent object that holds the object to which you want to read an attribute
 * @param objectName The name of the object to which the attribute is to be read
 * @param attributeName The name of the Attribute to read
 * @param data The memory to store the data
 * @return Standard HDF Error condition
 */
template <typename T> herr_t readVectorAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, QVector<T>& data)
{
  std::string objectNameStr = objectName.toStdString();
  std::string attributeNameStr = attributeName.toStdString();

  QVector<hsize_t> dims;
  H5T_class_t classType;
  size_t sizeType;
  hid_t typeID;
  herr_t err = getAttributeInfo(locationID, objectName, attributeName, dims, classType, sizeType, typeID) < 0;
  if(err < 0)
  {
    return err;
  }

  hsize_t numElements = std::accumulate(dims.cbegin(), dims.cend(), static_cast<hsize_t>(1), std::multiplies<hsize_t>());
  data.resize(static_cast<int>(numElements));

  err = H5Lite::readPointerAttribute(locationID, objectNameStr, attributeNameStr, data.data());
  return err;
}

/**
 * @brief Reads a scalar attribute value from a dataset
 * @param locationID
 * @param objectName The name of the dataset
 * @param attributeName The name of the Attribute
 * @param data The preallocated memory for the variable to be stored into
 * @return Standard HDF5 error condition
 */
template <typename T> herr_t readScalarAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, T& data)
{
  std::string objectNameStr = objectName.toStdString();
  std::string attributeNameStr = attributeName.toStdString();
  herr_t err = H5Lite::readScalarAttribute(locationID, objectNameStr, attributeNameStr, data);
  return err;
}

/**
 * @brief Reads the Attribute into a pre-allocated pointer
 * @param locationID
 * @param objectName The name of the dataset
 * @param attributeName The name of the Attribute
 * @param data The preallocated memory for the variable to be stored into
 * @return Standard HDF5 error condition
 */
template <typename T> herr_t readPointerAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, T* data)
{
  std::string objectNameStr = objectName.toStdString();
  std::string attributeNameStr = attributeName.toStdString();
  herr_t err = H5Lite::readPointerAttribute(locationID, objectNameStr, attributeNameStr, data);
  return err;
}

/**
 * @brief Reads a string attribute from an HDF object
 * @param locationID The Parent object that holds the object to which you want to read an attribute
 * @param objectName The name of the object to which the attribute is to be read
 * @param attributeName The name of the Attribute to read
 * @param data The memory to store the data
 * @return Standard HDF Error condition
 */
H5Support_EXPORT herr_t readStringAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, QString& data);

/**
 * @brief Reads a string attribute from an HDF object into a precallocated buffer
 * @param locationID The Parent object that holds the object to which you want to read an attribute
 * @param objectName The name of the object to which the attribute is to be read
 * @param attributeName The name of the Attribute to read
 * @param data The memory to store the data into
 * @return Standard HDF Error condition
 */
H5Support_EXPORT herr_t readStringAttribute(hid_t locationID, const QString& objectName, const QString& attributeName, char* data);
/**
 * @brief Returns the number of dimensions for a given attribute
 * @param locationID The HDF5 id of the parent group/file for the objectName
 * @param objectName The name of the dataset
 * @param attributeName The name of the attribute
 * @param rank (out) Number of dimensions is store into this variable
 */
H5Support_EXPORT herr_t getAttributeNDims(hid_t locationID, const QString& objectName, const QString& attributeName, hid_t& rank);

/**
 * @brief Returns the number of dimensions for a given dataset
 * @param locationID The HDF5 id of the parent group/file for the objectName
 * @param objectName The name of the dataset
 * @param rank (out) Number of dimensions is store into this variable
 */
H5Support_EXPORT herr_t getDatasetNDims(hid_t locationID, const QString& datasetName, hid_t& rank);

/**
 * @brief Returns the H5T value for a given dataset.
 *
 * Returns the type of data stored in the dataset. You MUST use H5Tclose(tid)
 * on the returned value or resource leaks will occur.
 * @param locationID A Valid HDF5 file or group id.
 * @param datasetName Path to the dataset
 * @return
 */
H5Support_EXPORT hid_t getDatasetType(hid_t locationID, const QString& datasetName);
}; // namespace QH5Lite

#if defined(H5Support_NAMESPACE)
}
#endif
