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
#pragma once

#include <QtCore/QtDebug>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Core/EbsdSetGetMacros.h"

#ifdef EbsdLib_ENABLE_HDF5
#include <hdf5.h>
#endif
/**
 * @class EbsdImporter EbsdImporter.h EbsdLib/EbsdImporter.h
 * @brief  This class is a pure virtual class that defines the interface that
 * subclasses should implement in order to write a conforming EBSD Data Importer
 *
 * @date Aug 8, 2011
 * @version 1.0
 */
class EbsdLib_EXPORT EbsdImporter
{
public:
  using Self = EbsdImporter;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer();

  /**
   * @brief Returns the name of the class for EbsdImporter
   */
  const QString getNameOfClass() const;
  /**
   * @brief Returns the name of the class for EbsdImporter
   */
  static QString ClassName();

  virtual ~EbsdImporter() = default;

  /**
   * @brief Sets an Error Message
   */
  EBSD_VIRTUAL_INSTANCE_STRING_PROPERTY(PipelineMessage);

  /**
   * @brief Sets an error condition
   */
  /**
   * @brief Setter property for ErrorCode
   */
  virtual void setErrorCode(int value)
  {
    m_ErrorCode = value;
  }
  /**
   * @brief Getter property for ErrorCode
   * @return Value of ErrorCode
   */
  virtual int getErrorCode() const
  {
    return m_ErrorCode;
  }

  /**
   * @brief Cancel the operation
   */
  /**
   * @brief Setter property for Cancel
   */
  virtual void setCancel(bool value)
  {
    m_Cancel = value;
  }

  /**
   * @brief Getter property for Cancel
   * @return Value of Cancel
   */
  virtual bool getCancel() const
  {
    return m_Cancel;
  }

  /**
   * @brief Either prints a message or sends the message to the User Interface
   * @param message The message to print
   * @param progress The progress of the Reconstruction normalized to a value between 0 and 100
   */
  virtual void progressMessage(const QString& message, int progress)
  {
    qDebug() << progress << "% " << message;
  }

  /**
   * @brief Either prints a message or sends the message to the User Interface
   * @param message The message to print
   * @param progress The progress of the Reconstruction normalized to a value between 0 and 100
   */
  virtual void progressMessage(const QString* message, int progress)
  {
    qDebug() << progress << "% " << *message;
  }

  /**
   * @brief Imports the EBSD file. This is a pure virtual method to force subclasses
   * to implement this method.
   * @param fildId HDF5 fileId of an open HDF5 file that the data will be stored into
   * @param index The integer index value of this EBSD data file
   * @param ebsdFile The raw data file from the manufacturere (.ang, .ctf)
   */
  virtual int importFile(hid_t fileId, int64_t index, const QString& ebsd) = 0;

  /**
   * @brief Returns the dimensions for the EBSD Data set
   * @param x Number of X Voxels (out)
   * @param y Number of Y Voxels (out)
   */
  virtual void getDims(int64_t& x, int64_t& y) = 0;

  /**
   * @brief Returns the x and y resolution of the voxels
   * @param x The x resolution (out)
   * @param y The y resolution (out)
   */
  virtual void getSpacing(float& x, float& y) = 0;

  /**
   * @brief Return the number of slices imported
   * @return
   */
  virtual int numberOfSlicesImported() = 0;

  /**
   * @brief This function sets the version of the H5Ebsd file that will be written.
   * @param version
   * @return
   */
  virtual void setFileVersion(uint32_t version) = 0;

protected:
  EbsdImporter() = default;

public:
  EbsdImporter(const EbsdImporter&) = delete;            // Copy Constructor Not Implemented
  EbsdImporter(EbsdImporter&&) = delete;                 // Move Constructor Not Implemented
  EbsdImporter& operator=(const EbsdImporter&) = delete; // Copy Assignment Not Implemented
  EbsdImporter& operator=(EbsdImporter&&) = delete;      // Move Assignment Not Implemented

private:
  int m_ErrorCode = 0;
  bool m_Cancel = false;
};
