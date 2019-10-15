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

#include <list>
#include <string>

#include <hdf5.h>

#include "H5Support/H5Support.h"

#if defined(H5Support_NAMESPACE)
namespace H5Support_NAMESPACE
{
#endif

/**
 * @brief General Utilities for working with the HDF5 data files and API
 */
namespace H5Utilities
{
enum class CustomHDFDataTypes : int32_t
{
  Group = 1,
  Dataset = 2,
  Type = 4,
  Link = 8,
  Any = 15
};

// -----------HDF5 File Operations
/**
 * @brief Opens a H5 file at path filename. Can be made read only access. Returns the id of the file object.
 * @param filename
 * @param readOnly
 * @return
 */
H5Support_EXPORT hid_t openFile(const std::string& filename, bool readOnly = false);

/**
 * @brief Creates a H5 file at path filename. Returns the id of the file object.
 * @param filename
 * @return
 */
H5Support_EXPORT hid_t createFile(const std::string& filename);

/**
 * @brief Closes a H5 file object. Returns the H5 error code.
 * @param fileID
 * @return
 */
H5Support_EXPORT herr_t closeFile(hid_t& fileID);

// -------------- HDF Indentifier Methods ----------------------------
/**
 * @brief Returns the path to an object
 * @param objectID The HDF5 id of the object
 * @param trim set to False to trim the path
 * @return  The path to the object relative to the objectID
 */
H5Support_EXPORT std::string getObjectPath(hid_t locationID, bool trim = false);

/**
 * @brief Returns the hdf object type
 * @param objectID The hdf5 object id
 * @param objectName The path to the data set
 * @param objectType The type of the object
 * @return  Negative value on error
 */
H5Support_EXPORT herr_t getObjectType(hid_t objectID, const std::string& objectName, int32_t& objectType);

/**
 * @brief Retrieves the object name for a given index
 * @param fileID The hdf5 object id
 * @param index The index to retrieve the name for
 * @param name The variable to store the name
 * @return Negative value is error
 */
H5Support_EXPORT herr_t objectNameAtIndex(hid_t fileID, int32_t index, std::string& name);

/**
 * @brief Returns the path to an object's parent
 * @param objectID The HDF5 id of the object
 * @param trim set to False to trim the path
 * @return  The path to the object relative to the objectID
 */
H5Support_EXPORT std::string getParentPath(hid_t objectID);

/**
 * @brief Returns the path to an object's parent
 * @param objectPath The HDF5 path to the object
 * @param trim set to False to trim the path
 * @return  The path to the object relative to the objectID
 */
H5Support_EXPORT std::string getParentPath(const std::string& objectPath);

/**
 * @brief Returns the object's name from object path
 * @param objectPath The HDF5 path to the object
 * @return  The object name
 */
H5Support_EXPORT std::string getObjectNameFromPath(const std::string& objectPath);

/**
 * @brief Returns if a given hdf5 object is a group
 * @param objectID The hdf5 object that contains an object with name objectName
 * @param objectName The name of the object to check
 * @return True if the given hdf5 object id is a group
 */
H5Support_EXPORT bool isGroup(hid_t nodeID, const std::string& objectName);

/**
 * @brief Opens an HDF5 object for hdf5 operations
 * @param locId the Object id of the parent
 * @param objectPath The path of the object to open
 * @return The hdf5 id of the opened object. Negative value is error.
 */
H5Support_EXPORT hid_t openHDF5Object(hid_t locationID, const std::string& objectName);

/**
 * @brief Closes the object id
 * @param locId The object id to close
 * @return Negative value is error.
 */
H5Support_EXPORT herr_t closeHDF5Object(hid_t objectID);

/**
 * @brief Returns the associated string for the given HDF class type.
 * @param classType
 * @return
 */
H5Support_EXPORT std::string HDFClassTypeAsStr(hid_t classType);

/**
 * @brief prints the class type of the given class
 * @param classT The Class Type to print
 */
H5Support_EXPORT void printHDFClassType(H5T_class_t classType);

// -------------- HDF Group Methods ----------------------------
/**
 * @brief Returns a list of child hdf5 objects for a given object id
 * @param locationID The parent hdf5 id
 * @param typeFilter A filter to apply to the list
 * @param names Variable to store the list
 * @return
 */
H5Support_EXPORT herr_t getGroupObjects(hid_t locationID, int32_t typeFilter, std::list<std::string>& names);

/**
 * @brief Creates a HDF Group by checking if the group already exists. If the
 * group already exists then that group is returned otherwise a new group is
 * created.
 * @param locationID The HDF unique id given to files or groups
 * @param group The name of the group to create. Note that this group name should
 * not be any sort of 'path'. It should be a single group.
 */
H5Support_EXPORT hid_t createGroup(hid_t locationID, const std::string& group);

/**
 * @brief Given a path relative to the Parent ID, this method will create all
 * the intermediate groups if necessary.
 * @param pathToCheck The path to either create or ensure exists.
 * @param parent The HDF unique id for the parent
 * @return Error Condition: Negative is error. Positive is success.
 */
H5Support_EXPORT hid_t createGroupsFromPath(const std::string& pathToCheck, hid_t parent);

/**
 * @brief Given a path relative to the Parent ID, this method will create all
 * the intermediate groups if necessary.
 * @param datasetPath The path to the dataset that you want to make all the intermediate groups for
 * @param parent The HDF unique id for the parent
 * @return Error Condition: Negative is error. Positive is success.
 */
H5Support_EXPORT hid_t createGroupsForDataset(const std::string& datasetPath, hid_t parent);

/**
 * @brief Extracts the object name from a given path
 * @param path The path which to extract the object name
 * @return The name of the object
 */
H5Support_EXPORT std::string extractObjectName(const std::string& path);

// -------------- HDF Attribute Methods ----------------------------
/**
 * @brief Looks for an attribute with a given name
 * @param locationID The objects Parent id
 * @param objectName The name of the object
 * @param attributeName The attribute to look for (by name)
 * @return True if the attribute exists.
 */
H5Support_EXPORT bool probeForAttribute(hid_t locationID, const std::string& objectName, const std::string& attributeName);

/**
 * @brief Returns a list of all the attribute names
 * @param objectID The parent object
 * @param names Variable to hold the list of attribute names
 * @return Negate value is error
 */
H5Support_EXPORT herr_t getAllAttributeNames(hid_t objectID, std::list<std::string>& results);

/**
 * @brief Returns a list of all the attribute names
 * @param objectID The parent object
 * @param objectName The name of the object whose attribute names you want a list
 * @param names Variable to hold the list of attribute names
 * @return Negative value is error
 */
H5Support_EXPORT herr_t getAllAttributeNames(hid_t locationID, const std::string& objectName, std::list<std::string>& names);
}; // namespace H5Utilities

#if defined(H5Support_NAMESPACE)
}
#endif
