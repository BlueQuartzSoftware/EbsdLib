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

#include "H5Lite.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>

namespace
{
constexpr size_t k_ChunkBase = 16 * 1024;
constexpr size_t k_ChunkMin = 8 * 1024;
constexpr size_t k_ChunkMax = 1024 * 1024;

/*-------------------------------------------------------------------------
 * Function: find_dataset
 *
 * Purpose: operator function used by H5LTfind_dataset
 *
 * Programmer: Pedro Vicente, pvn@ncsa.uiuc.edu
 *
 * Date: June 21, 2001
 *
 * Comments:
 *
 * Modifications:
 *
 *-------------------------------------------------------------------------
 */
herr_t find_dataset(hid_t /*locationID*/, const char* name, void* op_data)
{
  H5SUPPORT_MUTEX_LOCK()
  /* Define a default zero value for return. This will cause the iterator to continue if
   * the dataset is not found yet.
   */

  int32_t returnError = 0;

  char* datasetName = reinterpret_cast<char*>(op_data);

  /* Shut the compiler up */
  // locationID=locationID;

  /* Define a positive value for return value if the dataset was found. This will
   * cause the iterator to immediately return that positive value,
   * indicating short-circuit success
   */

  if(std::strcmp(name, datasetName) == 0)
  {
    returnError = 1;
  }

  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t find_attr(hid_t /*locationID*/, const char* name, const H5A_info_t* /*info*/, void* op_data)
{
  H5SUPPORT_MUTEX_LOCK()
  /* Define a default zero value for return. This will cause the iterator to continue if
   * the palette attribute is not found yet.
   */

  int32_t returnError = 0;

  char* attributeName = reinterpret_cast<char*>(op_data);

  /* Shut the compiler up */
  // locationID=locationID;

  /* Define a positive value for return value if the attribute was found. This will
   * cause the iterator to immediately return that positive value,
   * indicating short-circuit success
   */

  if(std::strcmp(name, attributeName) == 0)
  {
    returnError = 1;
  }

  return returnError;
}

template <class T, class Compare> constexpr const T& clamp(const T& v, const T& lo, const T& hi, Compare comp)
{
  return assert(!comp(hi, lo)), comp(v, lo) ? lo : comp(hi, v) ? hi : v;
}

template <class T> constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
  return clamp(v, lo, hi, std::less<>());
}

} // namespace

#if defined(H5Support_NAMESPACE)
using namespace H5Support_NAMESPACE;
#endif

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void H5Lite::disableErrorHandlers()
{
  H5SUPPORT_MUTEX_LOCK()

  HDF_ERROR_HANDLER_OFF;
}

// -----------------------------------------------------------------------------
//  Opens an ID for HDF5 operations
// -----------------------------------------------------------------------------
hid_t H5Lite::openId(hid_t locationID, const std::string& objectName, H5O_type_t objectType)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t objectID = -1;

  switch(objectType)
  {
  case H5O_TYPE_DATASET:

    /* Open the dataset. */
    if((objectID = H5Dopen(locationID, objectName.c_str(), H5P_DEFAULT)) < 0)
    {
      return -1;
    }
    break;

  case H5O_TYPE_GROUP:

    /* Open the group. */
    if((objectID = H5Gopen(locationID, objectName.c_str(), H5P_DEFAULT)) < 0)
    {
      return -1;
    }
    break;

  default:
    return -1;
  }

  return objectID;
}

// -----------------------------------------------------------------------------
//  Closes the given ID
// -----------------------------------------------------------------------------
herr_t H5Lite::closeId(hid_t objectID, int32_t objectType)
{
  H5SUPPORT_MUTEX_LOCK()

  switch(objectType)
  {
  case H5O_TYPE_DATASET:
    /* Close the dataset. */
    if(H5Dclose(objectID) < 0)
    {
      return -1;
    }
    break;

  case H5O_TYPE_GROUP:
    /* Close the group. */
    if(H5Gclose(objectID) < 0)
    {
      return -1;
    }
    break;

  default:
    return -1;
  }

  return 1;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string H5Lite::StringForHDFClassType(H5T_class_t classType)
{
  if(classType == H5T_NO_CLASS)
  {
    return "H5T_NO_CLASS";
  }
  if(classType == H5T_INTEGER)
  {
    return "H5T_INTEGER";
  }
  if(classType == H5T_FLOAT)
  {
    return "H5T_FLOAT";
  }
  if(classType == H5T_TIME)
  {
    return "H5T_TIME";
  }
  if(classType == H5T_STRING)
  {
    return "H5T_STRING";
  }
  if(classType == H5T_BITFIELD)
  {
    return "H5T_BITFIELD";
  }
  if(classType == H5T_OPAQUE)
  {
    return "H5T_OPAQUE";
  }
  if(classType == H5T_COMPOUND)
  {
    return "H5T_COMPOUND";
  }
  if(classType == H5T_REFERENCE)
  {
    return "H5T_REFERENCE";
  }
  if(classType == H5T_ENUM)
  {
    return "H5T_ENUM";
  }
  if(classType == H5T_VLEN)
  {
    return "H5T_VLEN";
  }
  if(classType == H5T_ARRAY)
  {
    return "H5T_ARRAY";
  }

  return "UNKNOWN";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
hid_t H5Lite::HDFTypeFromString(const std::string& value)
{
  H5SUPPORT_MUTEX_LOCK()

  if(value == "H5T_STRING")
  {
    return H5T_STRING;
  }

  if(value == "H5T_NATIVE_INT8")
  {
    return H5T_NATIVE_INT8;
  }
  if(value == "H5T_NATIVE_UINT8")
  {
    return H5T_NATIVE_UINT8;
  }

  if(value == "H5T_NATIVE_INT16")
  {
    return H5T_NATIVE_INT16;
  }
  if(value == "H5T_NATIVE_UINT16")
  {
    return H5T_NATIVE_UINT16;
  }

  if(value == "H5T_NATIVE_INT32")
  {
    return H5T_NATIVE_INT32;
  }
  if(value == "H5T_NATIVE_UINT32")
  {
    return H5T_NATIVE_UINT32;
  }

  if(value == "H5T_NATIVE_INT64")
  {
    return H5T_NATIVE_INT64;
  }
  if(value == "H5T_NATIVE_UINT64")
  {
    return H5T_NATIVE_UINT64;
  }

  if(value == "H5T_NATIVE_FLOAT")
  {
    return H5T_NATIVE_FLOAT;
  }
  if(value == "H5T_NATIVE_DOUBLE")
  {
    return H5T_NATIVE_DOUBLE;
  }

  std::cout << "Error: HDFTypeFromString - Unknown Type: " << value << std::endl;
  return -1;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string H5Lite::StringForHDFType(hid_t dataTypeIdentifier)
{
  H5SUPPORT_MUTEX_LOCK()

  if(dataTypeIdentifier == H5T_STRING)
  {
    return "H5T_STRING";
  }

  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_INT8) != 0)
  {
    return "H5T_NATIVE_INT8";
  }
  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_UINT8) != 0)
  {
    return "H5T_NATIVE_UINT8";
  }

  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_INT16) != 0)
  {
    return "H5T_NATIVE_INT16";
  }
  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_UINT16) != 0)
  {
    return "H5T_NATIVE_UINT16";
  }

  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_INT32) != 0)
  {
    return "H5T_NATIVE_INT32";
  }
  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_UINT32) != 0)
  {
    return "H5T_NATIVE_UINT32";
  }

  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_INT64) != 0)
  {
    return "H5T_NATIVE_INT64";
  }
  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_UINT64) != 0)
  {
    return "H5T_NATIVE_UINT64";
  }

  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_FLOAT) != 0)
  {
    return "H5T_NATIVE_FLOAT";
  }
  if(H5Tequal(dataTypeIdentifier, H5T_NATIVE_DOUBLE) != 0)
  {
    return "H5T_NATIVE_DOUBLE";
  }

  std::cout << "Error: HDFTypeForPrimitiveAsStr - Unknown Type: " << dataTypeIdentifier << std::endl;
  return "Unknown";
}

// -----------------------------------------------------------------------------
//  Finds an Attribute given an object to look in
// -----------------------------------------------------------------------------
herr_t H5Lite::findAttribute(hid_t locationID, const std::string& attributeName)
{
  H5SUPPORT_MUTEX_LOCK()

  hsize_t attributeNum;
  herr_t returnError = 0;

  attributeNum = 0;
  returnError = H5Aiterate(locationID, H5_INDEX_NAME, H5_ITER_INC, &attributeNum, find_attr, const_cast<char*>(attributeName.c_str()));

  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool H5Lite::datasetExists(hid_t locationID, const std::string& datasetName)
{
  H5SUPPORT_MUTEX_LOCK()

  H5O_info_t objectInfo{};
  HDF_ERROR_HANDLER_OFF
  herr_t error = H5Oget_info_by_name(locationID, datasetName.c_str(), &objectInfo, H5P_DEFAULT);
  HDF_ERROR_HANDLER_ON
  return error >= 0;
}

// -----------------------------------------------------------------------------
//  We assume a null terminated string
// -----------------------------------------------------------------------------
herr_t H5Lite::writeStringDataset(hid_t locationID, const std::string& datasetName, size_t size, const char* data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID = -1;
  hid_t dataspaceID = -1;
  hid_t typeID = -1;
  herr_t error = -1;
  herr_t returnError = 0;

  /* create a string data type */
  if((typeID = H5Tcopy(H5T_C_S1)) >= 0)
  {
    if(H5Tset_size(typeID, size) >= 0)
    {
      if(H5Tset_strpad(typeID, H5T_STR_NULLTERM) >= 0)
      {
        /* Create the data space for the dataset. */
        if((dataspaceID = H5Screate(H5S_SCALAR)) >= 0)
        {
          /* Create the dataset. */
          if((datasetID = H5Dcreate(locationID, datasetName.c_str(), typeID, dataspaceID, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT)) >= 0)
          {
            if(nullptr != data)
            {
              error = H5Dwrite(datasetID, typeID, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
              if(error < 0)
              {
                std::cout << "Error Writing String Data" << std::endl;
                returnError = error;
              }
            }
          }
          else
          {
            // returnError = datasetID;
            returnError = 0;
          }
          CloseH5D(datasetID, error, returnError, datasetName);
        }
        CloseH5S(dataspaceID, error, returnError);
      }
    }
    CloseH5T(typeID, error, returnError);
  }
  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t H5Lite::writeVectorOfStringsDataset(hid_t locationID, const std::string& datasetName, const std::vector<std::string>& data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t dataspaceID = -1;
  hid_t memSpace = -1;
  hid_t datatype = -1;
  hid_t datasetID = -1;
  herr_t error = -1;
  herr_t returnError = 0;

  std::array<hsize_t, 1> dims = {data.size()};
  if((dataspaceID = H5Screate_simple(static_cast<int>(dims.size()), dims.data(), nullptr)) >= 0)
  {
    dims[0] = 1;

    if((memSpace = H5Screate_simple(static_cast<int>(dims.size()), dims.data(), nullptr)) >= 0)
    {

      datatype = H5Tcopy(H5T_C_S1);
      H5Tset_size(datatype, H5T_VARIABLE);

      if((datasetID = H5Dcreate(locationID, datasetName.c_str(), datatype, dataspaceID, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT)) >= 0)
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
          const char* strPtr = element.c_str();
          error = H5Dwrite(datasetID, datatype, memSpace, dataspaceID, H5P_DEFAULT, &strPtr);
          if(error < 0)
          {
            std::cout << "Error Writing String Data: " __FILE__ << "(" << __LINE__ << ")" << std::endl;
            returnError = error;
          }
        }
        CloseH5D(datasetID, error, returnError, datasetName);
      }
      H5Tclose(datatype);
      CloseH5S(memSpace, error, returnError);
    }

    CloseH5S(dataspaceID, error, returnError);
  }
  return returnError;
}

// -----------------------------------------------------------------------------
//  Writes a string to a HDF5 dataset
// -----------------------------------------------------------------------------
herr_t H5Lite::writeStringDataset(hid_t locationID, const std::string& datasetName, const std::string& data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID = -1;
  hid_t dataspaceID = -1;
  hid_t typeID = -1;
  size_t size = 0;
  herr_t error = -1;
  herr_t returnError = 0;

  /* create a string data type */
  if((typeID = H5Tcopy(H5T_C_S1)) >= 0)
  {
    size = data.size() + 1;
    if(H5Tset_size(typeID, size) >= 0)
    {
      if(H5Tset_strpad(typeID, H5T_STR_NULLTERM) >= 0)
      {
        /* Create the data space for the dataset. */
        if((dataspaceID = H5Screate(H5S_SCALAR)) >= 0)
        {
          /* Create or open the dataset. */
          HDF_ERROR_HANDLER_OFF
          datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT);
          HDF_ERROR_HANDLER_ON
          if(datasetID < 0) // dataset does not exist so create it
          {
            datasetID = H5Dcreate(locationID, datasetName.c_str(), typeID, dataspaceID, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
          }

          if(datasetID >= 0)
          {
            if(!data.empty())
            {
              error = H5Dwrite(datasetID, typeID, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.c_str());
              if(error < 0)
              {
                std::cout << "Error Writing String Data" << std::endl;
                returnError = error;
              }
            }
          }
          else
          {
            // returnError = datasetID;
            returnError = 0;
          }
          CloseH5D(datasetID, error, returnError, datasetName);
          //          error = H5Dclose(datasetID);
          //          if (error < 0) {
          //            std::cout << "Error Closing Dataset." << std::endl;
          //            returnError = error;
          //          }
        }
        CloseH5S(dataspaceID, error, returnError);
        //        error = H5Sclose(dataspaceID);
        //        if ( error < 0) {
        //          std::cout << "Error closing Dataspace." << std::endl;
        //          returnError = error;
        //        }
      }
    }
    CloseH5T(typeID, error, returnError);
    //    error = H5Tclose(typeID);
    //    if (error < 0 ) {
    //     std::cout << "Error closing DataType" << std::endl;
    //     returnError = error;
    //    }
  }
  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t H5Lite::writeStringAttributes(hid_t locationID, const std::string& objectName, const std::map<std::string, std::string>& attributes)
{
  H5SUPPORT_MUTEX_LOCK()

  herr_t error = 0;
  for(const auto& attribute : attributes)
  {
    error = writeStringAttribute(locationID, objectName, attribute.first, attribute.second);
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
hsize_t H5Lite::getNumberOfElements(hid_t locationID, const std::string& datasetName)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t dataspaceID;
  hsize_t numElements = 0;
  datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT);
  if(datasetID < 0)
  {
    std::cout << "H5Lite.cpp::getNumberOfElements(" << __LINE__ << ") Error opening Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
    return -1;
  }
  if(datasetID >= 0)
  {
    dataspaceID = H5Dget_space(datasetID);
    if(dataspaceID > 0)
    {
      int32_t rank = H5Sget_simple_extent_ndims(dataspaceID);
      if(rank > 0)
      {
        std::vector<hsize_t> dims(rank, 0); // Allocate enough room for the dims
        error = H5Sget_simple_extent_dims(dataspaceID, dims.data(), nullptr);
        numElements = std::accumulate(dims.cbegin(), dims.cend(), static_cast<hsize_t>(1), std::multiplies<hsize_t>());
      }
      error = H5Sclose(dataspaceID);
      if(error < 0)
      {
        std::cout << "Error Closing Data Space" << std::endl;
        returnError = error;
      }
    }
    else
    {
      std::cout << "Error Opening SpaceID" << std::endl;
      // returnError = dataspaceID;
    }
    error = H5Dclose(datasetID);
    if(error < 0)
    {
      std::cout << "Error Closing Dataset" << std::endl;
      returnError = error;
    }
  }
  return numElements;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t H5Lite::writeStringAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, hsize_t size, const char* data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t attributeType;
  hid_t attributeSpaceID;
  hid_t attributeID;
  hid_t objectID;
  int32_t hasAttribute;
  H5O_info_t objectInfo{};
  size_t attributeSize;
  herr_t error = 0;
  herr_t returnError = 0;

  /* Get the type of object */
  returnError = H5Oget_info_by_name(locationID, objectName.c_str(), &objectInfo, H5P_DEFAULT);
  if(returnError >= 0)
  {
    /* Open the object */
    objectID = openId(locationID, objectName, objectInfo.type);
    if(objectID >= 0)
    {
      /* Create the attribute */
      attributeType = H5Tcopy(H5T_C_S1);
      if(attributeType >= 0)
      {
        attributeSize = size; /* extra null term */
        error = H5Tset_size(attributeType, attributeSize);
        if(error < 0)
        {
          std::cout << "Error Setting H5T Size" << std::endl;
          returnError = error;
        }
        if(error >= 0)
        {
          error = H5Tset_strpad(attributeType, H5T_STR_NULLTERM);
          if(error < 0)
          {
            std::cout << "Error adding a null terminator." << std::endl;
            returnError = error;
          }
          if(error >= 0)
          {
            attributeSpaceID = H5Screate(H5S_SCALAR);
            if(attributeSpaceID >= 0)
            {
              /* Verify if the attribute already exists */
              hasAttribute = findAttribute(objectID, attributeName);
              /* The attribute already exists, delete it */
              if(hasAttribute == 1)
              {
                error = H5Adelete(objectID, attributeName.c_str());
                if(error < 0)
                {
                  std::cout << "Error Deleting Attribute '" << attributeName << "' from Object '" << objectName << "'" << std::endl;
                  returnError = error;
                }
              }
              if(error >= 0)
              {
                /* Create and write the attribute */
                attributeID = H5Acreate(objectID, attributeName.c_str(), attributeType, attributeSpaceID, H5P_DEFAULT, H5P_DEFAULT);
                if(attributeID >= 0)
                {
                  error = H5Awrite(attributeID, attributeType, data);
                  if(error < 0)
                  {
                    std::cout << "Error Writing String attribute." << std::endl;

                    returnError = error;
                  }
                }
                CloseH5A(attributeID, error, returnError);
              }
              CloseH5S(attributeSpaceID, error, returnError);
            }
          }
        }
        CloseH5T(attributeType, error, returnError);
      }
      else
      {
        // returnError = attributeType;
      }
      /* Close the object */
      error = closeId(objectID, objectInfo.type);
      if(error < 0)
      {
        std::cout << "Error Closing Object Id" << std::endl;
        returnError = error;
      }
    }
  }
  return returnError;
}

// -----------------------------------------------------------------------------
//  Writes a string to an HDF5 Attribute
// -----------------------------------------------------------------------------
herr_t H5Lite::writeStringAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, const std::string& data)
{
  return writeStringAttribute(locationID, objectName, attributeName, data.size() + 1, data.data());
}

// -----------------------------------------------------------------------------
//  Reads a String dataset into a std::string
// -----------------------------------------------------------------------------
herr_t H5Lite::readStringDataset(hid_t locationID, const std::string& datasetName, std::string& data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID; // dataset id
  hid_t typeID;    // type id
  herr_t error = 0;
  herr_t returnError = 0;
  hsize_t size;
  data.clear();
  datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT);
  if(datasetID < 0)
  {
    std::cout << "H5Lite.cpp::readStringDataset(" << __LINE__ << ") Error opening Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
    return -1;
  }
  /*
   * Get the datatype.
   */
  typeID = H5Dget_type(datasetID);
  if(typeID >= 0)
  {
    htri_t isVariableString = H5Tis_variable_str(typeID); // Test if the string is variable length

    if(isVariableString == 1)
    {
      std::vector<std::string> strings;
      error = readVectorOfStringDataset(locationID, datasetName, strings); // Read the string
      if(error < 0 || (strings.size() > 1 && !strings.empty()))
      {
        std::cout << "Error Reading string dataset. There were multiple Strings and the program asked for a single string." << std::endl;
        returnError = error;
      }
      else
      {
        data.assign(strings[0]);
      }
    }
    else
    {
      size = H5Dget_storage_size(datasetID);
      std::vector<char> buffer(static_cast<size_t>(size + 1), 0x00); // Allocate and Zero and array
      error = H5Dread(datasetID, typeID, H5S_ALL, H5S_ALL, H5P_DEFAULT, buffer.data());
      if(error < 0)
      {
        std::cout << "Error Reading string dataset." << std::endl;
        returnError = error;
      }
      else
      {
        data.append(buffer.data()); // Append the string to the given string
      }
    }
  }
  CloseH5D(datasetID, error, returnError, datasetName);
  CloseH5T(typeID, error, returnError);
  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t H5Lite::readStringDataset(hid_t locationID, const std::string& datasetName, char* data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID; // dataset id
  hid_t typeID;    // type id
  herr_t error = 0;
  herr_t returnError = 0;

  datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT);
  if(datasetID < 0)
  {
    std::cout << "H5Lite.cpp::readStringDataset(" << __LINE__ << ") Error opening Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
    return -1;
  }
  typeID = H5Dget_type(datasetID);
  if(typeID >= 0)
  {
    error = H5Dread(datasetID, typeID, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
    if(error < 0)
    {
      std::cout << "Error Reading string dataset." << std::endl;
      returnError = error;
    }
    CloseH5T(typeID, error, returnError);
  }
  CloseH5D(datasetID, error, returnError, datasetName);
  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t H5Lite::readVectorOfStringDataset(hid_t locationID, const std::string& datasetName, std::vector<std::string>& data)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID; // dataset id
  hid_t typeID;    // type id
  herr_t error = 0;
  herr_t returnError = 0;

  datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT);
  if(datasetID < 0)
  {
    std::cout << "H5Lite.cpp::readVectorOfStringDataset(" << __LINE__ << ") Error opening Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
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
    int nDims = H5Sget_simple_extent_dims(dataspaceID, dims, nullptr);
    if(nDims != 1)
    {
      CloseH5S(dataspaceID, error, returnError);
      CloseH5T(typeID, error, returnError);
      std::cout << "H5Lite.cpp::readVectorOfStringDataset(" << __LINE__ << ") Number of dims should be 1 but it was " << nDims << ". Returning early. Is your data file correct?" << std::endl;
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
      std::cout << "H5Lite.cpp::readVectorOfStringDataset(" << __LINE__ << ") Error reading Dataset at locationID (" << locationID << ") with object name (" << datasetName << ")" << std::endl;
      return -3;
    }
    /*
     * copy the data into the vector of strings
     */
    data.resize(dims[0]);
    for(size_t i = 0; i < dims[0]; i++)
    {
      // printf("%s[%d]: %s\n", "VlenStrings", i, rData[i].p);
      data[i] = std::string(rData[i]);
    }
    /*
     * Close and release resources.  Note that H5Dvlen_reclaim works
     * for variable-length strings as well as variable-length arrays.
     * Also note that we must still free the array of pointers stored
     * in rData, as H5Tvlen_reclaim only frees the data these point to.
     */
    status = H5Dvlen_reclaim(memtype, dataspaceID, H5P_DEFAULT, rData.data());
    CloseH5S(dataspaceID, error, returnError);
    CloseH5T(typeID, error, returnError);
    CloseH5T(memtype, error, returnError);
  }

  CloseH5D(datasetID, error, returnError, datasetName);

  return returnError;
}

// -----------------------------------------------------------------------------
//  Reads a string Attribute from the HDF file
// -----------------------------------------------------------------------------
herr_t H5Lite::readStringAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, std::string& data)
{
  H5SUPPORT_MUTEX_LOCK()

  /* identifiers */
  hid_t objectID;
  H5O_info_t objectInfo{};
  hid_t attributeID;
  hid_t attributeType;
  std::vector<char> attributeOutput;
  hsize_t size;
  herr_t error = 0;
  herr_t returnError = 0;
  data.clear();
  HDF_ERROR_HANDLER_OFF;

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
    hid_t attrTypeId = H5Aget_type(attributeID);
    htri_t isVariableString = H5Tis_variable_str(attrTypeId); // Test if the string is variable length
    H5Tclose(attrTypeId);
    if(isVariableString == 1)
    {
      data.clear();
      returnError = -1;
      CloseH5A(attributeID, error, returnError);
      return returnError;
    }
    if(attributeID >= 0)
    {
      size = H5Aget_storage_size(attributeID);
      attributeOutput.resize(static_cast<size_t>(size)); // Resize the vector to the proper length
      attributeType = H5Aget_type(attributeID);
      if(attributeType >= 0)
      {
        error = H5Aread(attributeID, attributeType, attributeOutput.data());
        if(error < 0)
        {
          std::cout << "Error Reading Attribute." << std::endl;
          returnError = error;
        }
        else
        {
          if(attributeOutput[size - 1] == 0) // null Terminated string
          {
            size -= 1;
          }
          data.append(attributeOutput.data(), size); // Append the data to the passed in string
        }
        CloseH5T(attributeType, error, returnError);
      }
      CloseH5A(attributeID, error, returnError);
    }
    else
    {
      // returnError = attributeID;
    }
    error = closeId(objectID, objectInfo.type);
    if(error < 0)
    {
      std::cout << "Error Closing Object ID" << std::endl;
      returnError = error;
    }
  }
  HDF_ERROR_HANDLER_ON;
  return returnError;
}

// -----------------------------------------------------------------------------
//  Reads a string Attribute from the HDF file
// -----------------------------------------------------------------------------
herr_t H5Lite::readStringAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName, char* data)
{
  H5SUPPORT_MUTEX_LOCK()

  /* identifiers */
  hid_t objectID;
  H5O_info_t objectInfo{};
  hid_t attributeID;
  hid_t attributeType;
  herr_t error = 0;
  herr_t returnError = 0;

  HDF_ERROR_HANDLER_OFF;

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
      attributeType = H5Aget_type(attributeID);
      if(attributeType >= 0)
      {
        error = H5Aread(attributeID, attributeType, data);
        if(error < 0)
        {
          std::cout << "Error Reading Attribute." << std::endl;
          returnError = error;
        }
        CloseH5T(attributeType, error, returnError);
      }
      CloseH5A(attributeID, error, returnError);
    }
    else
    {
      // returnError = attributeID;
    }
    error = closeId(objectID, objectInfo.type);
    if(error < 0)
    {
      std::cout << "Error Closing Object ID" << std::endl;
      returnError = error;
    }
  }
  HDF_ERROR_HANDLER_ON;
  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t H5Lite::getDatasetNDims(hid_t locationID, const std::string& datasetName, hid_t& rank)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID;
  hid_t dataspaceID;
  herr_t error = 0;
  herr_t returnError = 0;
  rank = 0;

  /* Open the dataset. */
  if((datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT)) < 0)
  {
    return -1;
  }

  /* Get the dataspace handle */
  dataspaceID = H5Dget_space(datasetID);
  if(dataspaceID >= 0)
  {

    /* Get rank */
    rank = H5Sget_simple_extent_ndims(dataspaceID);
    if(rank < 0)
    {
      // returnError = rank;
      rank = 0;
      std::cout << "Error Getting the rank of the dataset:" << std::endl;
    }

    /* Terminate access to the dataspace */
    CloseH5S(dataspaceID, error, returnError);
  }

  /* End access to the dataset */
  error = H5Dclose(datasetID);
  if(error < 0)
  {
    returnError = error;
    rank = 0;
  }
  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
herr_t H5Lite::getAttributeNDims(hid_t locationID, const std::string& objectName, const std::string& attributeName, hid_t& rank)
{
  H5SUPPORT_MUTEX_LOCK()

  /* identifiers */
  hid_t objectID;
  H5O_info_t objectInfo{};
  hid_t attributeID;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t dataspaceID;
  rank = -1;
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
      dataspaceID = H5Aget_space(attributeID);
      if(dataspaceID >= 0)
      {
        rank = H5Sget_simple_extent_ndims(dataspaceID);
        CloseH5S(dataspaceID, error, returnError);
      }
      CloseH5A(attributeID, error, returnError);
    }
    else
    {
      returnError = static_cast<herr_t>(attributeID);
    }
    error = closeId(objectID, objectInfo.type);
    if(error < 0)
    {
      std::cout << "Error Closing Object ID" << std::endl;
      returnError = error;
    }
  }

  return returnError;
}

// -----------------------------------------------------------------------------
//  Returns the type of data stored in the dataset. You MUST use H5Tclose(typeID)
//  on the returned value or resource leaks will occur.
// -----------------------------------------------------------------------------
hid_t H5Lite::getDatasetType(hid_t locationID, const std::string& datasetName)
{
  H5SUPPORT_MUTEX_LOCK()

  herr_t error = 0;
  herr_t returnError = 0;
  hid_t datasetID = -1;
  /* Open the dataset. */
  if((datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT)) < 0)
  {
    return -1;
  }
  /* Get an identifier for the datatype. */
  hid_t typeID = H5Dget_type(datasetID);
  CloseH5D(datasetID, error, returnError, datasetName);
  if(returnError < 0)
  {
    return static_cast<hid_t>(returnError);
  }
  return typeID;
}

// -----------------------------------------------------------------------------
//  Get the dataset information
// -----------------------------------------------------------------------------
herr_t H5Lite::getDatasetInfo(hid_t locationID, const std::string& datasetName, std::vector<hsize_t>& dims, H5T_class_t& classType, size_t& sizeType)
{
  H5SUPPORT_MUTEX_LOCK()

  hid_t datasetID;
  hid_t typeID;
  hid_t dataspaceID;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t rank = 0;

  /* Open the dataset. */
  if((datasetID = H5Dopen(locationID, datasetName.c_str(), H5P_DEFAULT)) < 0)
  {
    return -1;
  }

  /* Get an identifier for the datatype. */
  typeID = H5Dget_type(datasetID);
  if(typeID >= 0)
  {
    /* Get the class. */
    classType = H5Tget_class(typeID);
    /* Get the size. */
    sizeType = H5Tget_size(typeID);
    /* Release the datatype. */
    error = H5Tclose(typeID);
    if(error < 0)
    {
      std::cout << "Error Closing H5Type" << std::endl;
      returnError = error;
    }
  }
  /* Get the dataspace handle */
  dataspaceID = H5Dget_space(datasetID);
  if(dataspaceID >= 0)
  {
    /* Get the Number of Dimensions */
    rank = H5Sget_simple_extent_ndims(dataspaceID);
    if(rank > 0)
    {
      std::vector<hsize_t> _dims(rank, 0);
      /* Get dimensions */
      error = H5Sget_simple_extent_dims(dataspaceID, _dims.data(), nullptr);
      if(error < 0)
      {
        std::cout << "Error Getting Simple Extents for dataset" << std::endl;
        returnError = error;
      }
      // Copy the dimensions into the dims vector
      dims.clear(); // Erase everything in the Vector
      std::copy(_dims.cbegin(), _dims.cend(), std::back_inserter(dims));
    }
    else if(classType == H5T_STRING)
    {
      dims.clear(); // Erase everything in the Vector
      dims.push_back(sizeType);
    }
    /* Terminate access to the dataspace */
    CloseH5S(dataspaceID, error, returnError);
  }

  /* End access to the dataset */
  CloseH5D(datasetID, error, returnError, datasetName);
  return returnError;
}

// -----------------------------------------------------------------------------
//  You must close the attributeType argument or resource leaks will occur. Use
//  H5Tclose(typeID); after your call to this method if you do not need the id for
//   anything.
// -----------------------------------------------------------------------------
herr_t H5Lite::getAttributeInfo(hid_t locationID, const std::string& objectName, const std::string& attributeName, std::vector<hsize_t>& dims, H5T_class_t& typeClass, size_t& typeSize, hid_t& typeID)
{
  H5SUPPORT_MUTEX_LOCK()

  /* identifiers */
  hid_t objectID;
  H5O_info_t objectInfo{};
  hid_t attributeID;
  herr_t error = 0;
  herr_t returnError = 0;
  hid_t dataspaceID;
  hid_t rank = -1;

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
      /* Get an identifier for the datatype. */
      typeID = H5Aget_type(attributeID);
      if(typeID > 0)
      {
        /* Get the class. */
        typeClass = H5Tget_class(typeID);
        /* Get the size. */
        typeSize = H5Tget_size(typeID);
        dataspaceID = H5Aget_space(attributeID);
        if(dataspaceID >= 0)
        {
          if(typeClass == H5T_STRING)
          {
            rank = 1;
            dims.resize(rank);
            dims[0] = typeSize;
          }
          else
          {
            rank = H5Sget_simple_extent_ndims(dataspaceID);
            std::vector<hsize_t> _dims(rank, 0);
            /* Get dimensions */
            error = H5Sget_simple_extent_dims(dataspaceID, _dims.data(), nullptr);
            if(error < 0)
            {
              std::cout << "Error Getting Attribute dims" << std::endl;
              returnError = error;
            }
            // Copy the dimensions into the dims vector
            dims.clear(); // Erase everything in the Vector
            dims.resize(rank);
            std::copy(_dims.cbegin(), _dims.cend(), dims.begin());
          }
          CloseH5S(dataspaceID, error, returnError);
          dataspaceID = 0;
        }
      }
      CloseH5A(attributeID, error, returnError);
      attributeID = 0;
    }
    else
    {
      returnError = -1;
    }
    error = closeId(objectID, objectInfo.type);
    if(error < 0)
    {
      std::cout << "Error Closing Object ID" << std::endl;
      returnError = error;
    }
  }
  return returnError;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::vector<hsize_t> H5Lite::guessChunkSize(const std::vector<hsize_t>& dims, size_t typeSize)
{
  std::vector<hsize_t> chunks(dims.cbegin(), dims.cend());
  size_t chunksSize = chunks.size();

  hsize_t product = std::accumulate(chunks.cbegin(), chunks.cend(), static_cast<hsize_t>(1), std::multiplies<hsize_t>());
  hsize_t datasetSize = product * typeSize;
  double percentage = std::pow(2.0, std::log10(static_cast<double>(datasetSize) / (1024.0 * 1024.0)));
  hsize_t targetSize = static_cast<hsize_t>(static_cast<double>(k_ChunkBase) * percentage);
  targetSize = clamp(targetSize, static_cast<hsize_t>(k_ChunkMin), static_cast<hsize_t>(k_ChunkMax));

  size_t index = 0;

  bool foundChunkSize = false;

  while(!foundChunkSize)
  {
    product = std::accumulate(chunks.cbegin(), chunks.cend(), static_cast<hsize_t>(1), std::multiplies<hsize_t>());
    hsize_t chunkBytes = product * typeSize;
    if(chunkBytes < targetSize)
    {
      foundChunkSize = true;
      break;
    }

    if(chunkBytes < k_ChunkMax && static_cast<double>(chunkBytes - targetSize) / static_cast<double>(targetSize) < 0.5)
    {
      foundChunkSize = true;
      break;
    }

    if(product == 1)
    {
      foundChunkSize = true;
      break;
    }

    size_t i = index % chunksSize;

    chunks[i] = static_cast<hsize_t>(std::ceil(static_cast<double>(chunks[i]) / 2.0));
    ++index;
  }

  return chunks;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::vector<hsize_t> H5Lite::guessChunkSize(int32_t rank, const hsize_t* dims, size_t typeSize)
{
  std::vector<hsize_t> vDims(rank, 0);
  std::copy(dims, dims + rank, vDims.data());
  return guessChunkSize(vDims, typeSize);
}
