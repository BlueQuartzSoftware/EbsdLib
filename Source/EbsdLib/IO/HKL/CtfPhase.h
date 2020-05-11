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

#include <cstring>

#include <QtCore/QVector>
#include <QtCore/QString>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Core/EbsdSetGetMacros.h"
#include "EbsdLib/Core/EbsdLibConstants.h"

#include "CtfConstants.h"

/**
 * @class CtfPhase CtfPhase.h EbsdLib/IO/HKL/CtfPhase.h
 * @brief This class holds all the values for a "Phase" header block in an HKL .ctf file
 *
 * @date Mar 23, 2011
 * @version 1.0
 */
class EbsdLib_EXPORT CtfPhase
{
public:
  using Self = CtfPhase;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer();

  EBSD_STATIC_NEW_MACRO(CtfPhase)
  /**
   * @brief Returns the name of the class for CtfPhase
   */
  QString getNameOfClass() const;
  /**
   * @brief Returns the name of the class for CtfPhase
   */
  static QString ClassName();

  virtual ~CtfPhase();

  EBSD_INSTANCE_PROPERTY(int, PhaseIndex)

  EBSD_INSTANCE_PROPERTY(QVector<float>, LatticeConstants) // 1x6 array
  /**
   * @brief Setter property for PhaseName
   */
  void setPhaseName(const QString& value);
  /**
   * @brief Getter property for PhaseName
   * @return Value of PhaseName
   */
  QString getPhaseName() const;

  EBSD_INSTANCE_PROPERTY(EbsdLib::Ctf::LaueGroupTable, LaueGroup) // <== Laue Group

  EBSD_INSTANCE_PROPERTY(int, SpaceGroup)
  /**
   * @brief Setter property for Internal1
   */
  void setInternal1(const QString& value);
  /**
   * @brief Getter property for Internal1
   * @return Value of Internal1
   */
  QString getInternal1() const;

  /**
   * @brief Setter property for Internal2
   */
  void setInternal2(const QString& value);
  /**
   * @brief Getter property for Internal2
   * @return Value of Internal2
   */
  QString getInternal2() const;

  /**
   * @brief Setter property for Comment
   */
  void setComment(const QString& value);
  /**
   * @brief Getter property for Comment
   * @return Value of Comment
   */
  QString getComment() const;

  /**
   * @brief Parses a header line into a CtfPhase class
   */
  void parsePhase(QByteArray& line);

  /**
   * @brief Prints some debugging info about this class
   */
  void printSelf(std::ostream& stream);

  /**
   * @brief Returns the type of crystal structure for this phase.
   */
  unsigned int determineLaueGroup();

  QString getMaterialName();

protected:
  CtfPhase();

  void convertEuropeanDecimals(QByteArray& line);

public:
  CtfPhase(const CtfPhase&) = delete;            // Copy Constructor Not Implemented
  CtfPhase(CtfPhase&&) = delete;                 // Move Constructor Not Implemented
  CtfPhase& operator=(const CtfPhase&) = delete; // Copy Assignment Not Implemented
  CtfPhase& operator=(CtfPhase&&) = delete;      // Move Assignment Not Implemented

private:
  QString m_PhaseName = {};
  QString m_Internal1 = {};
  QString m_Internal2 = {};
  QString m_Comment = {};
};
