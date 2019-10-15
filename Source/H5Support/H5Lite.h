/* ============================================================================
 * Copyright (c) 2007-2019 BlueQuartz Software, LLC
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
 *    United States Air Force Prime Contract FA8650-04-C-5229
 *    United States Air Force Prime Contract FA8650-15-D-5231
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#pragma once

#include <array>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <typeinfo>
#include <vector>

#include <hdf5.h>

#include "H5Support/H5Macros.h"
#include "H5Support/H5Support.h"

/**
 * @brief Namespace to bring together some high level methods to read/write data to HDF5 files.
 * @author Mike Jackson
 * @date April 2007
 * @version $Revision: 1.3 $
 */
namespace H5Lite
{
/**
 * @brief Turns off the global error handler/reporting objects. Note that once
 * they are turned off using this method they CAN NOT be turned back on. If you
 * would like to turn them off for a piece of code then surround your code with
 * the HDF_ERROR_HANDLER_OFF and HDF_ERROR_HANDLER_ON macros defined in
 * H5Lite.h
 */
H5Support_EXPORT void disableErrorHandlers();

/**
 * @brief Opens an object for HDF5 operations
 * @param locationID The parent object that holds the true object we want to open
 * @param objectName The string name of the object
 * @param objectType The HDF5_TYPE of object
 * @return Standard HDF5 Error Conditions
 */
H5Support_EXPORT hid_t openId(hid_t locationID, const std::string& objectName, H5O_type_t objectType);

/**
 * @brief Opens an HDF5 Object
 * @param objectID The Object id
 * @param objectType Basic Object Type
 * @return Standard HDF5 Error Conditions
 */
H5Support_EXPORT herr_t closeId(hid_t objectID, int32_t objectType);

/**
 * @brief Returns a string of the HDF datatype
 * @param classType
 * @return
 */
H5Support_EXPORT std::string StringForHDFClassType(H5T_class_t classType);

/**
 * @brief Given one of the HDF Types as a string, this will return the HDF Type
 * as an hid_t value.
 * @param value The HDF_Type as a string
 * @return the hid_t value for the given type. -1 if the string does not match a type.
 */
H5Support_EXPORT hid_t HDFTypeFromString(const std::string& value);

/**
 * @brief Returns a string version of the HDF Type
 * @param type The HDF5 Type to query
 * @return
 */
H5Support_EXPORT std::string StringForHDFType(hid_t dataTypeIdentifier);

/**
 * @brief Returns the HDF Type for a given primitive value.
 * @param value A value to use. Can be anything. Just used to get the type info
 * from
 * @return A std::string representing the HDF5 Type
 */
template <typename T> std::string HDFTypeForPrimitiveAsStr(T value)
{
  H5SUPPORT_MUTEX_LOCK()

  if(typeid(value) == typeid(int8_t))
  {
    return "H5T_NATIVE_INT8";
  }
  if(typeid(value) == typeid(uint8_t))
  {
    return "H5T_NATIVE_UINT8";
  }

  if(typeid(value) == typeid(int16_t))
  {
    return "H5T_NATIVE_INT16";
  }
  if(typeid(value) == typeid(uint16_t))
  {
    return "H5T_NATIVE_UINT16";
  }

  if(typeid(value) == typeid(int32_t))
  {
    return "H5T_NATIVE_INT32";
  }
  if(typeid(value) == typeid(uint32_t))
  {
    return "H5T_NATIVE_UINT32";
  }

  if(typeid(value) == typeid(int64_t))
  {
    return "H5T_NATIVE_INT64";
  }
  if(typeid(value) == typeid(uint64_t))
  {
    return "H5T_NATIVE_UINT64";
  }

  if(typeid(value) == typeid(float))
  {
    return "H5T_NATIVE_FLOAT";
  }
  if(typeid(value) == typeid(double))
  {
    return "H5T_NATIVE_DOUBLE";
  }

  // if (typeid(value) == typeid(bool)) return "H5T_NATIVE_UINT8";

  std::cout << "Error: HDFTypeForPrimitiveAsStr - Unknown Type: " << typeid(value).name() << std::endl;
  return "";
}

/**
 * @brief Returns the HDF Type for a given primitive value.
 * @param value A value to use. Can be anything. Just used to get the type info
 * from
 * @return The HDF5 native type for the value
 */
template <typename T> hid_t HDFTypeForPrimitive(T value)
{
  H5SUPPORT_MUTEX_LOCK()

  if(typeid(value) == typeid(float))
  {
    return H5T_NATIVE_FLOAT;
  }
  if(typeid(value) == typeid(double))
  {
    return H5T_NATIVE_DOUBLE;
  }

  if(typeid(value) == typeid(int8_t))
  {
    return H5T_NATIVE_INT8;
  }
  if(typeid(value) == typeid(uint8_t))
  {
    return H5T_NATIVE_UINT8;
  }
#if CMP_TYPE_CHAR_IS_SIGNED
  if(typeid(value) == typeid(char))
  {
    return H5T_NATIVE_INT8;
  }
#else
  if(typeid(value) == typeid(char))
  {
    return H5T_NATIVE_UINT8;
  }
#endif
  if(typeid(value) == typeid(signed char))
  {
    return H5T_NATIVE_INT8;
  }
  if(typeid(value) == typeid(unsigned char))
  {
    return H5T_NATIVE_UINT8;
  }

  if(typeid(value) == typeid(int16_t))
  {
    return H5T_NATIVE_INT16;
  }
  if(typeid(value) == typeid(short))
  {
    return H5T_NATIVE_INT16;
  }
  if(typeid(value) == typeid(signed short))
  {
    return H5T_NATIVE_INT16;
  }
  if(typeid(value) == typeid(uint16_t))
  {
    return H5T_NATIVE_UINT16;
  }
  if(typeid(value) == typeid(unsigned short))
  {
    return H5T_NATIVE_UINT16;
  }

  if(typeid(value) == typeid(int32_t))
  {
    return H5T_NATIVE_INT32;
  }
  if(typeid(value) == typeid(uint32_t))
  {
    return H5T_NATIVE_UINT32;
  }
#if(CMP_SIZEOF_INT == 4)
  if(typeid(value) == typeid(int))
  {
    return H5T_NATIVE_INT32;
  }
  if(typeid(value) == typeid(signed int))
  {
    return H5T_NATIVE_INT32;
  }
  if(typeid(value) == typeid(unsigned int))
  {
    return H5T_NATIVE_UINT32;
  }
#endif

#if(CMP_SIZEOF_LONG == 4)
  if(typeid(value) == typeid(long int))
  {
    return H5T_NATIVE_INT32;
  }
  if(typeid(value) == typeid(signed long int))
  {
    return H5T_NATIVE_INT32;
  }
  if(typeid(value) == typeid(unsigned long int))
  {
    return H5T_NATIVE_UINT32;
  }
#elif(CMP_SIZEOF_LONG == 8)
  if(typeid(value) == typeid(long int))
  {
    return H5T_NATIVE_INT64;
  }
  if(typeid(value) == typeid(signed long int))
  {
    return H5T_NATIVE_INT64;
  }
  if(typeid(value) == typeid(unsigned long int))
  {
    return H5T_NATIVE_UINT64;
  }
#endif

#if(CMP_SIZEOF_LONG_LONG == 8)
  if(typeid(value) == typeid(long long int))
  {
    return H5T_NATIVE_INT64;
  }
  if(typeid(value) == typeid(signed long long int))
  {
    return H5T_NATIVE_INT64;
  }
  if(typeid(value) == typeid(unsigned long long int))
  {
    return H5T_NATIVE_UINT64;
  }
#endif
  if(typeid(value) == typeid(int64_t))
  {
    return H5T_NATIVE_INT64;
  }
  if(typeid(value) == typeid(uint64_t))
  {
    return H5T_NATIVE_UINT64;
  }

  if(typeid(value) == typeid(bool))
  {
    return H5T_NATIVE_UINT8;
  }

  std::cout << "Error: HDFTypeForPrimitive - Unknown Type: " << (typeid(value).name()) << std::endl;
  const char* name = typeid(value).name();
  if(nullptr != name && name[0] == 'l')
  {
    std::cout << "You are using 'long int' as a type which is not 32/64 bit safe. Suggest you use one of the H5SupportTypes defined in <Common/H5SupportTypes.h> such as int32_t or uint32_t."
              << std::endl;
  }
  return -1;
}

/**
 * @brief Inquires if an attribute named attributeName exists attached to the object locationID.
 * @param locationID The location to search
 * @param attributeName The attribute to search for
 * @return Standard HDF5 Error condition
 */
H5Support_EXPORT herr_t findAttribute(hid_t locationID, const std::string& attributeName);

/**
 * @brief Finds a Data set given a data set name
 * @param locationID The location to search
 * @param datasetName The dataset to search for
 * @return Standard HDF5 Error condition. Negative=DataSet
 */
H5Support_EXPORT bool datasetExists(hid_t locationID, const std::string& datasetName);

/**
 * @brief Writes the data of a pointer to an HDF5 file
 * @param locationID The hdf5 object id of the parent
 * @param datasetName The name of the dataset to write to. This can be a name of Path
 * @param rank The number of dimensions
 * @param dims The sizes of each dimension
 * @param data The data to be written.
 * @return Standard hdf5 error condition.
 */
template <typename T> herr_t writePointerDataset(hid_t locationID, const std::string& datasetName, int32_t rank, const hsize_t* dims, const T* data)
{
  H5SUPPORT_MUTEX_LOCK()

  herr_t error = -1;
  hid_t datasetID = -1;
  hid_t dataspaceID = -1;
  herr_t returnError = 0;

  if(nullptr == data)
  {
    return -2;
  }
  hid_t dataType = HDFTypeForPrimitive(data[0]);
  if(dataType == -1)
  {
    return -1;
  }
  // Create the DataSpace
  dataspaceID = H5Screate_simple(rank, dims, nullptr);
  if(dataspaceID < 0)
  {
    return static_cast<herr_t>(dataspaceID);
  }
  // Create the Dataset
  // This will fail if datasetName contains a "/"!
  datasetID = H5Dcreate(locationID, datasetName.c_str(), dataType, dataspaceID, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  if(datasetID >= 0)
  {
    error = H5Dwrite(datasetID, dataType, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
    if(error < 0)
    {
      std::cout << "Error Writing Data '" << datasetName << "'" << std::endl;
      std::cout << "    rank = " << rank << std::endl;
      uint64_t totalSize = 1;
      for(size_t i = 0; i < rank; ++i)
      {
        std::cout << "    dim[" << i << "] = " << dims[i] << std::endl;
        totalSize = totalSize * dims[i];
      }
      std::cout << "    Total Elements = " << totalSize << std::endl;
      std::cout << "    Size of Type (Bytes) = " << sizeof(T) << std::endl;
      std::cout << "    Total Bytes to Write =  " << (sizeof(T) * totalSize) << std::endl;
      returnError = error;
    }
    error = H5Dclose(datasetID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataset." << std::endl;
      returnError = error;
    }
  }
  else
  {
    returnError = static_cast<herr_t>(datasetID);
  }
  /* Terminate access to the data space. */
  error = H5Sclose(dataspaceID);
  if(error < 0)
  {
    std::cout << "Error Closing Dataspace" << std::endl;
    returnError = error;
  }
  return returnError;
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
template <typename T> static herr_t replacePointerDataset(hid_t locationID, const std::string& datasetName, int32_t rank, const hsize_t* dims, const T* data)
{
  H5SUPPORT_MUTEX_LOCK()

  herr_t error = -1;
  hid_t datasetID = -1;
  hid_t dataspaceID = -1;
  herr_t returnError = 0;

  if(data == nullptr)
  {
    return -2;
  }

  hid_t dataType = H5Lite::HDFTypeForPrimitive(data[0]);
  if(dataType == -1)
  {
    return -1;
  }
  // Create the DataSpace
  dataspaceID = H5Screate_simple(rank, dims, nullptr);
  if(dataspaceID < 0)
  {
    return dataspaceID;
  }

  HDF_ERROR_HANDLER_OFF
  datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT);
  HDF_ERROR_HANDLER_ON
  if(datasetID < 0) // dataset does not exist so create it
  {
    datasetID = H5Dcreate(locationID, datasetName.c_str(), dataType, dataspaceID, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  }
  if(datasetID >= 0)
  {
    error = H5Dwrite(datasetID, dataType, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
    if(error < 0)
    {
      std::cout << "Error Writing Data" << std::endl;
      returnError = error;
    }
    error = H5Dclose(datasetID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataset." << std::endl;
      returnError = error;
    }
  }
  else
  {
    returnError = static_cast<herr_t>(datasetID);
  }
  /* Terminate access to the data space. */
  error = H5Sclose(dataspaceID);
  if(error < 0)
  {
    std::cout << "Error Closing Dataspace" << std::endl;
    returnError = error;
  }
  return returnError;
}

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
 * dimensions array. By using a std::vector<hsize_t> that stores the values of
 * each of the dimensions we can reduce the number of arguments to this method as
 * the value of the "rank" simply becomes dims.length(). So to create a Dims variable
 * for a 3D data space of size(x,y,z) = {10,20,30} I would use the following code:
 * <code>
 * std::vector<hsize_t> dims;
 * dims.push_back(10);
 * dims.push_back(20);
 * dims.push_back(30);
 * </code>
 *
 * Also when passing data BE SURE that the type of data and the data type match.
 * For example if I create some data in a std::vector<UInt8Type> I would need to
 * pass H5T_NATIVE_UINT8 as the dataType.
 */
template <typename T> herr_t writeVectorDataset(hid_t locationID, const std::string& datasetName, const std::vector<hsize_t>& dims, const std::vector<T>& data)
{
  return writePointerDataset(locationID, datasetName, static_cast<int32_t>(dims.size()), dims.data(), data.data());
}

/**
 * @brief Returns a guess for the vector of chunk dimensions based on the input parameters.
 * @param rank The number of dimensions
 * @param dims The dimensions of the dataset
 * @param typeSize The size of the data type for the dataset
 * @return The vector of chunk dimensions guess
 */
H5Support_EXPORT std::vector<hsize_t> guessChunkSize(int32_t rank, const hsize_t* dims, size_t typeSize);

/**
 * @brief Returns a guess for the vector of chunk dimensions based on the input parameters.
 * @param dims The vector dimensions of the dataset
 * @param typeSize The size of the data type for the dataset
 * @return The vector of chunk dimensions guess
 */
H5Support_EXPORT std::vector<hsize_t> guessChunkSize(const std::vector<hsize_t>& dims, size_t typeSize);

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
herr_t writePointerDatasetCompressed(hid_t locationID, const std::string& datasetName, int32_t rank, const hsize_t* dims, const T* data, int32_t cRank, const hsize_t* cDims, int32_t compressionLevel)
{
  H5SUPPORT_MUTEX_LOCK()

  herr_t error = -1;
  herr_t returnError = 0;

  if(data == nullptr)
  {
    return -100;
  }

  hid_t dataType = HDFTypeForPrimitive(data[0]);

  if(dataType == -1)
  {
    return -101;
  }

  // Create the DataSpace

  hid_t dataspaceID = H5Screate_simple(rank, dims, nullptr);
  if(dataspaceID < 0)
  {
    return -102;
  }

  // Create property list for chunking and compression

  hid_t propertListID = H5Pcreate(H5P_DATASET_CREATE);
  if(propertListID < 0)
  {
    returnError = -103;
    error = H5Sclose(dataspaceID);
    if(error < 0)
    {
      returnError = -104;
    }
    return returnError;
  }

  error = H5Pset_chunk(propertListID, cRank, cDims);
  if(error < 0)
  {
    error = H5Pclose(propertListID);
    if(error < 0)
    {
      returnError = -105;
    }
    error = H5Sclose(dataspaceID);
    if(error < 0)
    {
      returnError = -106;
    }
    return returnError;
  }

  error = H5Pset_deflate(propertListID, compressionLevel);
  if(error < 0)
  {
    error = H5Pclose(propertListID);
    if(error < 0)
    {
      returnError = -107;
    }
    error = H5Sclose(dataspaceID);
    if(error < 0)
    {
      returnError = -108;
    }
    return returnError;
  }

  // Create the Dataset

  hid_t datasetID = H5Dcreate(locationID, datasetName.c_str(), dataType, dataspaceID, H5P_DEFAULT, propertListID, H5P_DEFAULT);
  if(datasetID >= 0)
  {
    error = H5Dwrite(datasetID, dataType, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
    if(error < 0)
    {
      std::cout << "Error Writing Data" << std::endl;
      returnError = -108;
    }
    error = H5Dclose(datasetID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataset." << std::endl;
      returnError = -110;
    }
  }
  else
  {
    returnError = -111;
  }

  // Terminate access to the data space and property list.

  error = H5Pclose(propertListID);
  if(error < 0)
  {
    std::cout << "Error Closing Property List" << std::endl;
    returnError = -112;
  }
  error = H5Sclose(dataspaceID);
  if(error < 0)
  {
    std::cout << "Error Closing Dataspace" << std::endl;
    returnError = -113;
  }

  return returnError;
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
herr_t writeVectorDatasetCompressed(hid_t locationID, const std::string& datasetName, const std::vector<hsize_t>& dims, const std::vector<T>& data, const std::vector<hsize_t>& cDims,
                                    int32_t compressionLevel)
{
  return writePointerDatasetCompressed(locationID, datasetName, static_cast<int32_t>(dims.size()), dims.data(), data.data(), static_cast<int32_t>(cDims.size()), cDims.data(), compressionLevel);
}
#endif

/**
 * @brief Creates a Dataset with the given name at the location defined by locationID from a std::array
 *
 * @param locationID The Parent location to store the data
 * @param datasetName The name of the dataset
 * @param dims The dimensions of the dataset
 * @param data The data to write to the file
 * @return Standard HDF5 error conditions
 */
template <typename T, size_t _Size> herr_t writeArrayDataset(hid_t locationID, const std::string& datasetName, const std::vector<hsize_t>& dims, const std::array<T, _Size>& data)
{
  return writePointerDataset(locationID, datasetName, static_cast<int32_t>(dims.size()), dims.data(), data.data());
}

/**
 * @brief Creates a Dataset with the given name at the location defined by locationID.
 * This version of writeDataset should be used with a single scalar value. If you
 * need to write an array of values, use the form that takes an std::vector<>
 *
 * @param locationID The Parent location to store the data
 * @param datasetName The name of the dataset
 * @param value The value to write to the HDF5 dataset
 * @return Standard HDF5 error conditions
 */
template <typename T> herr_t writeScalarDataset(hid_t locationID, const std::string& datasetName, const T& value)
{
  H5SUPPORT_MUTEX_LOCK()

  herr_t error = -1;
  hid_t datasetID = -1;
  hid_t dataspaceID = -1;
  herr_t returnError = 0;
  hsize_t dims = 1;
  hid_t rank = 1;
  hid_t dataType = HDFTypeForPrimitive(value);
  if(dataType == -1)
  {
    return -1;
  }
  // Create the DataSpace
  dataspaceID = H5Screate_simple(static_cast<int>(rank), &(dims), nullptr);
  if(dataspaceID < 0)
  {
    return static_cast<herr_t>(dataspaceID);
  }
  // Create the Dataset
  datasetID = H5Dcreate(locationID, datasetName.c_str(), dataType, dataspaceID, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  if(datasetID >= 0)
  {
    error = H5Dwrite(datasetID, dataType, H5S_ALL, H5S_ALL, H5P_DEFAULT, &value);
    if(error < 0)
    {
      std::cout << "Error Writing Data" << std::endl;
      returnError = error;
    }
    error = H5Dclose(datasetID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataset." << std::endl;
      returnError = error;
    }
  }
  else
  {
    returnError = static_cast<herr_t>(datasetID);
  }
  /* Terminate access to the data space. */
  error = H5Sclose(dataspaceID);
  if(error < 0)
  {
    std::cout << "Error Closing Dataspace" << std::endl;
    returnError = error;
  }
  return returnError;
}

/**
 * @brief Writes a std::string as a HDF Dataset.
 * @param locationID The Parent location to write the dataset
 * @param datasetName The Name to use for the dataset
 * @param data The actual data to write as a null terminated string
 * @return Standard HDF5 error conditions
 */
H5Support_EXPORT herr_t writeStringDataset(hid_t locationID, const std::string& datasetName, const std::string& data);

/**
 * @brief Writes a null terminated 'C String' to an HDF Dataset.
 * @param locationID The Parent location to write the dataset
 * @param datasetName The Name to use for the dataset
 * @param size The number of characters in the string
 * @param data const char pointer to write as a null terminated string
 * @return Standard HDF5 error conditions
 */
H5Support_EXPORT herr_t writeStringDataset(hid_t locationID, const std::string& datasetName, size_t size, const char* data);

/**
 * @brief Writes a vector of null terminated strings to an HDF dataset.
 * @param locationID
 * @param datasetName
 * @param size
 * @param data
 * @return
 */
H5Support_EXPORT herr_t writeVectorOfStringsDataset(hid_t locationID, const std::string& datasetName, const std::vector<std::string>& data);
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
template <typename T> herr_t writePointerAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, int32_t rank, const hsize_t* dims, const T* data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t objectID, dataspaceID, attributeID;
  herr_t hasAttribute;
  H5O_info_t objectInfo;
  herr_t error = 0;
  herr_t returnError = 0;
  T test = 0x00;
  hid_t dataType = HDFTypeForPrimitive(test);
  if(dataType == -1)
  {
    std::cout << "dataType was unknown" << std::endl;
    return -1;
  }
  /* Get the type of object */

  if(H5Oget_info_by_name(locationID, objectName.c_str(), &objectInfo, H5P_DEFAULT) < 0)
  {
    std::cout << "Error getting object info at locationID (" << locationID << ") with object name (" << objectName << ")" << std::endl;
    return -1;
  }
  /* Open the object */
  objectID = openId(locationID, objectName, objectInfo.type);
  if(objectID < 0)
  {
    std::cout << "Error opening Object for Attribute operations." << std::endl;
    return -1;
  }

  dataspaceID = H5Screate_simple(rank, dims, nullptr);
  if(dataspaceID >= 0)
  {
    /* Verify if the attribute already exists */
    hasAttribute = findAttribute(objectID, attributeName);

    /* The attribute already exists, delete it */
    if(hasAttribute == 1)
    {
      error = H5Adelete(objectID, attributeName.c_str());
      if(error < 0)
      {
        std::cout << "Error Deleting Existing Attribute" << std::endl;
        returnError = error;
      }
    }

    if(error >= 0)
    {
      /* Create the attribute. */
      attributeID = H5Acreate(objectID, attributeName.c_str(), dataType, dataspaceID, H5P_DEFAULT, H5P_DEFAULT);
      if(attributeID >= 0)
      {
        /* Write the attribute data. */
        error = H5Awrite(attributeID, dataType, data);
        if(error < 0)
        {
          std::cout << "Error Writing Attribute" << std::endl;
          returnError = error;
        }
      }
      /* Close the attribute. */
      error = H5Aclose(attributeID);
      if(error < 0)
      {
        std::cout << "Error Closing Attribute" << std::endl;
        returnError = error;
      }
    }
    /* Close the dataspace. */
    error = H5Sclose(dataspaceID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataspace" << std::endl;
      returnError = error;
    }
  }
  else
  {
    returnError = static_cast<herr_t>(dataspaceID);
  }
  /* Close the object */
  error = closeId(objectID, objectInfo.type);
  if(error < 0)
  {
    std::cout << "Error Closing HDF5 Object ID" << std::endl;
    returnError = error;
  }
  return returnError;
}

/**
 * @brief Writes an Attribute to an HDF5 Object
 * @param locationID The Parent Location of the HDFobject that is getting the attribute
 * @param objectName The Name of Object to write the attribute into.
 * @param attributeName The Name of the Attribute
 * @param dims The Dimensions of the data set
 * @param data The Attribute Data to write
 * @return Standard HDF Error Condition
 */
template <typename T> herr_t writeVectorAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, const std::vector<hsize_t>& dims, const std::vector<T>& data)
{
  return writePointerAttribute(locationID, objectName, attributeName, static_cast<int32_t>(dims.size()), dims.data(), data.data());
}

/**
 * @brief Writes a string as a null terminated attribute.
 * @param locationID The location to look for objectName
 * @param objectName The Object to write the attribute to
 * @param attributeName The name of the Attribute
 * @param data The string to write as the attribute
 * @return Standard HDF error conditions
 */
H5Support_EXPORT herr_t writeStringAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, const std::string& data);
/**
 * @brief Writes a null terminated string as an attribute
 * @param locationID The location to look for objectName
 * @param objectName The Object to write the attribute to
 * @param attributeName The name of the Attribute
 * @param size The number of characters  in the string
 * @param data pointer to a const char array
 * @return Standard HDF error conditions
 */
H5Support_EXPORT herr_t writeStringAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, hsize_t size, const char* data);

/**
 * @brief Writes attributes that all have a data type of STRING. The first value
 * in each set is the key, the second is the actual value of the attribute.
 * @param locationID The location to look for objectName
 * @param objectName The Object to write the attribute to
 * @param attributes The attributes to be written where the first value is the name
 * of the attribute, and the second is the actual value of the attribute.
 * @return Standard HDF error condition
 */
H5Support_EXPORT herr_t writeStringAttributes(hid_t locationID, const std::string& objectName, const std::map<std::string, std::string>& attributes);

/**
 * @brief Returns the total number of elements in the supplied dataset
 * @param locationID The parent location that contains the dataset to read
 * @param datasetName The name of the dataset to read
 * @return Number of elements in dataset
 */
H5Support_EXPORT hsize_t getNumberOfElements(hid_t locationID, const std::string& datasetName);

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
template <typename T> herr_t writeScalarAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, T data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t objectID, dataspaceID, attributeID;
  herr_t hasAttribute;
  H5O_info_t objectInfo;
  herr_t error = 0;
  herr_t returnError = 0;
  hsize_t dims = 1;
  int32_t rank = 1;
  hid_t dataType = HDFTypeForPrimitive(data);
  if(dataType == -1)
  {
    return -1;
  }
  /* Get the type of object */
  error = H5Oget_info_by_name(locationID, objectName.c_str(), &objectInfo, H5P_DEFAULT);
  if(error < 0)
  {
    std::cout << "Error getting object info at locationID (" << locationID << ") with object name (" << objectName << ")" << std::endl;
    return error;
  }
  /* Open the object */
  objectID = openId(locationID, objectName, objectInfo.type);
  if(objectID < 0)
  {
    std::cout << "Error opening Object for Attribute operations." << std::endl;
    return static_cast<herr_t>(objectID);
  }

  /* Create the data space for the attribute. */
  dataspaceID = H5Screate_simple(rank, &dims, nullptr);
  if(dataspaceID >= 0)
  {
    /* Verify if the attribute already exists */
    hasAttribute = findAttribute(objectID, attributeName);

    /* The attribute already exists, delete it */
    if(hasAttribute == 1)
    {
      error = H5Adelete(objectID, attributeName.c_str());
      if(error < 0)
      {
        std::cout << "Error Deleting Existing Attribute" << std::endl;
        returnError = error;
      }
    }

    if(error >= 0)
    {
      /* Create the attribute. */
      attributeID = H5Acreate(objectID, attributeName.c_str(), dataType, dataspaceID, H5P_DEFAULT, H5P_DEFAULT);
      if(attributeID >= 0)
      {
        /* Write the attribute data. */
        error = H5Awrite(attributeID, dataType, &data);
        if(error < 0)
        {
          std::cout << "Error Writing Attribute" << std::endl;
          returnError = error;
        }
      }
      /* Close the attribute. */
      error = H5Aclose(attributeID);
      if(error < 0)
      {
        std::cout << "Error Closing Attribute" << std::endl;
        returnError = error;
      }
    }
    /* Close the dataspace. */
    error = H5Sclose(dataspaceID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataspace" << std::endl;
      returnError = error;
    }
  }
  else
  {
    returnError = static_cast<herr_t>(dataspaceID);
  }

  /* Close the object */
  error = closeId(objectID, objectInfo.type);
  if(error < 0)
  {
    std::cout << "Error Closing HDF5 Object ID" << std::endl;
    returnError = error;
  }
  return returnError;
}

/**
 * @brief Reads data from the HDF5 File into a preallocated array.
 * @param locationID The parent location that contains the dataset to read
 * @param datasetName The name of the dataset to read
 * @param data A Pointer to the PreAllocated Array of Data
 * @return Standard HDF error condition
 */
template <typename T> herr_t readPointerDataset(hid_t locationID, const std::string& datasetName, T* data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t dataType = 0;
  T test = 0x00;
  dataType = HDFTypeForPrimitive(test);
  if(dataType == -1)
  {
    std::cout << "dataType was not supported." << std::endl;
    return -10;
  }
  if(locationID < 0)
  {
    std::cout << "locationID was Negative: This is not allowed." << std::endl;
    return -2;
  }
  if(nullptr == data)
  {
    std::cout << "The Pointer to hold the data is nullptr. This is NOT allowed." << std::endl;
    return -3;
  }
  datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT);
  if(datasetID < 0)
  {
    std::cout << " Error opening Dataset: " << datasetID << std::endl;
    return -1;
  }
  if(datasetID >= 0)
  {
    error = H5Dread(datasetID, dataType, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
    if(error < 0)
    {
      std::cout << "Error Reading Data." << std::endl;
      returnError = error;
    }
    error = H5Dclose(datasetID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataset id" << std::endl;
      returnError = error;
    }
  }
  return returnError;
}

/**
 * @brief Reads data from the HDF5 File into an std::vector<T> object. If the dataset
 * is very large this can be an expensive method to use. It is here for convenience
 * using STL with hdf5.
 * @param locationID The parent location that contains the dataset to read
 * @param datasetName The name of the dataset to read
 * @param data A std::vector<T>. Note the vector WILL be resized to fit the data.
 * The best idea is to just allocate the vector but not to size it. The method
 * will size it for you.
 * @return Standard HDF error condition
 */
template <typename T> herr_t readVectorDataset(hid_t locationID, const std::string& datasetName, std::vector<T>& data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t spaceId;
  hid_t dataType;
  T test = static_cast<T>(0x00);
  dataType = HDFTypeForPrimitive(test);
  if(dataType == -1)
  {
    return -1;
  }
  datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT);
  if(datasetID < 0)
  {
    std::cout << "H5Lite.h::readVectorDataset(" << __LINE__ << ") Error opening Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
    return -1;
  }
  if(datasetID >= 0)
  {
    spaceId = H5Dget_space(datasetID);
    if(spaceId > 0)
    {
      int32_t rank = H5Sget_simple_extent_ndims(spaceId);
      if(rank > 0)
      {
        std::vector<hsize_t> dims(rank, 0); // Allocate enough room for the dims
        error = H5Sget_simple_extent_dims(spaceId, dims.data(), nullptr);
        hsize_t numElements = std::accumulate(dims.cbegin(), dims.cend(), static_cast<hsize_t>(1), std::multiplies<hsize_t>());
        // std::cout << "NumElements: " << numElements << std::endl;
        // Resize the vector
        data.resize(numElements);
        error = H5Dread(datasetID, dataType, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.data());
        if(error < 0)
        {
          std::cout << "Error Reading Data.'" << datasetName << "'" << std::endl;
          returnError = error;
        }
      }
      error = H5Sclose(spaceId);
      if(error < 0)
      {
        std::cout << "Error Closing Data Space" << std::endl;
        returnError = error;
      }
    }
    else
    {
      std::cout << "Error Opening SpaceID" << std::endl;
      returnError = static_cast<herr_t>(spaceId);
    }
    error = H5Dclose(datasetID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataset" << std::endl;
      returnError = error;
    }
  }
  return returnError;
}

/**
 * @brief Reads a dataset that consists of a single scalar value
 * @param locationID The HDF5 file or group id
 * @param datasetName The name or path to the dataset to read
 * @param data The variable to store the data into
 * @return HDF error condition.
 */
template <typename T> herr_t readScalarDataset(hid_t locationID, const std::string& datasetName, T& data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID = 0;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t spaceId = 0;

  hid_t dataType = HDFTypeForPrimitive(data);
  if(dataType == -1)
  {
    return -1;
  }
  /* Open the dataset. */
  datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT);
  if(datasetID < 0)
  {
    std::cout << "H5Lite.h::readScalarDataset(" << __LINE__ << ") Error opening Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
    return -1;
  }
  if(datasetID >= 0)
  {
    spaceId = H5Dget_space(datasetID);
    if(spaceId > 0)
    {
      error = H5Dread(datasetID, dataType, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data);
      if(error < 0)
      {
        std::cout << "Error Reading Data at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
        returnError = error;
      }

      error = H5Sclose(spaceId);
      if(error < 0)
      {
        std::cout << "Error Closing Data Space at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
        returnError = error;
      }
    }
    else
    {
      returnError = static_cast<herr_t>(spaceId);
    }
    error = H5Dclose(datasetID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
      returnError = error;
    }
  }
  return returnError;
}

/**
 * @brief Reads a string dataset into the supplied string. Any data currently in the 'data' variable
 * is cleared first before the new data is read into the string.
 * @param locationID The parent group that holds the data object to read
 * @param datasetName The name of the dataset.
 * @param data The std::string to hold the data
 * @return Standard HDF error condition
 */
H5Support_EXPORT herr_t readStringDataset(hid_t locationID, const std::string& datasetName, std::string& data);

/**
 * @brief reads a null terminated string dataset into the supplied buffer. The buffer
 * should be already preallocated.
 * @param locationID The parent group that holds the data object to read
 * @param datasetName The name of the dataset.
 * @param data pointer to the buffer
 * @return Standard HDF error condition
 */
H5Support_EXPORT herr_t readStringDataset(hid_t locationID, const std::string& datasetName, char* data);

/**
 * @brief Reads a dataset of multiple strings into a std::vector<std::string>
 * @param locationID
 * @param datasetName
 * @param data
 * @return
 */
H5Support_EXPORT herr_t readVectorOfStringDataset(hid_t locationID, const std::string& datasetName, std::vector<std::string>& data);

/**
 * @brief Returns the information about an attribute.
 * You must close the typeID argument or resource leaks will occur. Use
 *  H5Tclose(typeID); after your call to this method if you do not need the id for
 *   anything.
 * @param locationID The parent location of the Dataset
 * @param objectName The name of the dataset
 * @param attributeName The name of the attribute
 * @param dims A std::vector that will hold the sizes of the dimensions
 * @param typeClass The HDF5 class type
 * @param typeSize THe HDF5 size of the data
 * @param typeID The Attribute ID - which needs to be closed after you are finished with the data
 * @return
 */
H5Support_EXPORT herr_t getAttributeInfo(hid_t locationID, const std::string& objectName, const std::string& attributeName, std::vector<hsize_t>& dims, H5T_class_t& typeClass, size_t& typeSize,
                                         hid_t& typeID);

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
template <typename T> herr_t readVectorAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, std::vector<T>& data)
{
  H5SUPPORT_MUTEX_LOCK()

  /* identifiers */
  hid_t objectID;
  H5O_info_t objectInfo;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t attributeID;
  hid_t typeID;
  T test = 0x00;
  hid_t dataType = HDFTypeForPrimitive(test);
  if(dataType == -1)
  {
    return -1;
  }
  // std::cout << "   Reading Vector Attribute at Path '" << objectName << "' with Key: '" << attributeName << "'" << std::endl;
  /* Get the type of object */
  error = H5Oget_info_by_name(locationID, objectName.c_str(), &objectInfo, H5P_DEFAULT);
  if(error < 0)
  {
    return error;
  }
  /* Open the object */
  objectID = openId(locationID, objectName, objectInfo.type);
  if(objectID >= 0)
  {
    attributeID = H5Aopen_by_name(locationID, objectName.c_str(), attributeName.c_str(), H5P_DEFAULT, H5P_DEFAULT);
    if(attributeID >= 0)
    {
      // Need to allocate the array size
      H5T_class_t typeClass;
      size_t typeSize;
      std::vector<hsize_t> dims;
      error = getAttributeInfo(locationID, objectName, attributeName, dims, typeClass, typeSize, typeID);
      hsize_t numElements = std::accumulate(dims.cbegin(), dims.cend(), static_cast<hsize_t>(1), std::multiplies<hsize_t>());
      // std::cout << "    Vector Attribute has " << numElements << " elements." << std::endl;
      data.resize(numElements);
      error = H5Aread(attributeID, dataType, data.data());
      if(error < 0)
      {
        std::cout << "Error Reading Attribute." << error << std::endl;
        returnError = error;
      }
      error = H5Aclose(attributeID);
      if(error < 0)
      {
        std::cout << "Error Closing Attribute" << std::endl;
        returnError = error;
      }
    }
    else
    {
      returnError = static_cast<herr_t>(attributeID);
    }
    error = closeId(objectID, objectInfo.type);
    if(error < 0)
    {
      std::cout << "Error Closing Object" << std::endl;
      returnError = error;
    }
  }
  return returnError;
}

/**
 * @brief Reads a scalar attribute value from a dataset
 * @param locationID
 * @param objectName The name of the dataset
 * @param attributeName The name of the Attribute
 * @param data The preallocated memory for the variable to be stored into
 * @return Standard HDF5 error condition
 */
template <typename T> herr_t readScalarAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, T& data)
{
  H5SUPPORT_MUTEX_LOCK()

  /* identifiers */
  hid_t objectID;
  H5O_info_t objectInfo;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t attributeID;
  T test = 0x00;
  hid_t dataType = HDFTypeForPrimitive(test);
  if(dataType == -1)
  {
    return -1;
  }
  // std::cout << "Reading Scalar style Attribute at Path '" << objectName << "' with Key: '" << attributeName << "'" << std::endl;
  /* Get the type of object */
  error = H5Oget_info_by_name(locationID, objectName.c_str(), &objectInfo, H5P_DEFAULT);
  if(error < 0)
  {
    return error;
  }
  /* Open the object */
  objectID = openId(locationID, objectName, objectInfo.type);
  if(objectID >= 0)
  {
    attributeID = H5Aopen_by_name(locationID, objectName.c_str(), attributeName.c_str(), H5P_DEFAULT, H5P_DEFAULT);
    if(attributeID >= 0)
    {
      error = H5Aread(attributeID, dataType, &data);
      if(error < 0)
      {
        std::cout << "Error Reading Attribute." << std::endl;
        returnError = error;
      }
      error = H5Aclose(attributeID);
      if(error < 0)
      {
        std::cout << "Error Closing Attribute" << std::endl;
        returnError = error;
      }
    }
    else
    {
      returnError = static_cast<herr_t>(attributeID);
    }
    error = closeId(objectID, objectInfo.type);
    if(error < 0)
    {
      std::cout << "Error Closing Object" << std::endl;
      returnError = error;
    }
  }
  return returnError;
}

/**
 * @brief Reads the Attribute into a pre-allocated pointer
 * @param locationID
 * @param objectName The name of the dataset
 * @param attributeName The name of the Attribute
 * @param data The preallocated memory for the variable to be stored into
 * @return Standard HDF5 error condition
 */
template <typename T> herr_t readPointerAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, T* data)
{
  H5SUPPORT_MUTEX_LOCK()

  /* identifiers */
  hid_t objectID;
  H5O_info_t objectInfo;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t attributeID;
  T test = 0x00;
  hid_t dataType = HDFTypeForPrimitive(test);
  if(dataType == -1)
  {
    return -1;
  }
  // std::cout << "   Reading Vector Attribute at Path '" << objectName << "' with Key: '" << attributeName << "'" << std::endl;
  /* Get the type of object */
  error = H5Oget_info_by_name(locationID, objectName.c_str(), &objectInfo, H5P_DEFAULT);
  if(error < 0)
  {
    return error;
  }
  /* Open the object */
  objectID = openId(locationID, objectName, objectInfo.type);
  if(objectID >= 0)
  {
    attributeID = H5Aopen_by_name(locationID, objectName.c_str(), attributeName.c_str(), H5P_DEFAULT, H5P_DEFAULT);
    if(attributeID >= 0)
    {
      error = H5Aread(attributeID, dataType, data);
      if(error < 0)
      {
        std::cout << "Error Reading Attribute." << error << std::endl;
        returnError = error;
      }
      error = H5Aclose(attributeID);
      if(error < 0)
      {
        std::cout << "Error Closing Attribute" << std::endl;
        returnError = error;
      }
    }
    else
    {
      returnError = static_cast<herr_t>(attributeID);
    }
    error = closeId(objectID, objectInfo.type);
    if(error < 0)
    {
      std::cout << "Error Closing Object" << std::endl;
      returnError = error;
    }
  }
  return returnError;
}

/**
 * @brief Reads a string attribute from an HDF object
 * @param locationID The Parent object that holds the object to which you want to read an attribute
 * @param objectName The name of the object to which the attribute is to be read
 * @param attributeName The name of the Attribute to read
 * @param data The memory to store the data
 * @return Standard HDF Error condition
 */
H5Support_EXPORT herr_t readStringAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, std::string& data);

/**
 * @brief Reads a string attribute from an HDF object into a precallocated buffer
 * @param locationID The Parent object that holds the object to which you want to read an attribute
 * @param objectName The name of the object to which the attribute is to be read
 * @param attributeName The name of the Attribute to read
 * @param data The memory to store the data into
 * @return Standard HDF Error condition
 */
H5Support_EXPORT herr_t readStringAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, char* data);
/**
 * @brief Returns the number of dimensions for a given attribute
 * @param locationID The HDF5 id of the parent group/file for the objectName
 * @param objectName The name of the dataset
 * @param attributeName The name of the attribute
 * @param rank (out) Number of dimensions is store into this variable
 */
H5Support_EXPORT herr_t getAttributeNDims(hid_t locationID, const std::string& objectName, const std::string& attributeName, hid_t& rank);

/**
 * @brief Returns the number of dimensions for a given dataset
 * @param locationID The HDF5 id of the parent group/file for the objectName
 * @param objectName The name of the dataset
 * @param rank (out) Number of dimensions is store into this variable
 */
H5Support_EXPORT herr_t getDatasetNDims(hid_t locationID, const std::string& objectName, hid_t& rank);

/**
 * @brief Returns the H5T value for a given dataset.
 *
 * Returns the type of data stored in the dataset. You MUST use H5Tclose(typeID)
 * on the returned value or resource leaks will occur.
 * @param locationID A Valid HDF5 file or group id.
 * @param datasetName Path to the dataset
 * @return
 */
H5Support_EXPORT hid_t getDatasetType(hid_t locationID, const std::string& datasetName);

/**
 * @brief Get the information about a dataset.
 *
 * @param locationID The parent location of the Dataset
 * @param datasetName The name of the dataset
 * @param dims A std::vector that will hold the sizes of the dimensions
 * @param typeClass The HDF5 class type
 * @param typeSize THe HDF5 size of the data
 * @return Negative value is Failure. Zero or Positive is success;
 */
H5Support_EXPORT herr_t getDatasetInfo(hid_t locationID, const std::string& datasetName, std::vector<hsize_t>& dims, H5T_class_t& classType, size_t& sizeType);
}; // namespace H5Lite

#if defined(H5Support_NAMESPACE)
}
#endif
