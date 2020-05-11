/* ============================================================================
 * Copyright (c) 2019 BlueQuartz Software, LLC
 * All rights reserved.
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
 * Neither the names of any of the BlueQuartz Software contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#pragma once

#include <vector>

#include <QtCore/QString>

#include "EbsdLib/EbsdLib.h"

namespace EbsdLib
{
/**
 * @class ToolTipGenerator ToolTipGenerator.h SIMPLib/Utilities/ToolTipGenerator.h
 * @brief The ToolTipGenerator class is used for generating HTML table strings.
 * Data is added one row at a time, and then data is converted into HTML upon request.
 * The default backround color is based on existing tool tips at the time the class
 * is created, but it can be freely changed up to HTML generation.
 */
class EbsdLib_EXPORT ToolTipGenerator
{
  struct EbsdLib_EXPORT RowItem
  {
    enum class Type
    {
      Title,
      Value,
      Spacer
    };
    Type type = Type::Spacer;
    QString name;
    QString value;
  };

public:
  /**
   * @brief Default constructor
   */
  ToolTipGenerator();

  /**
   * @brief Destructor
   */
  virtual ~ToolTipGenerator();

  /**
   * @brief Adds a new title row.
   * @param title
   */
  void addTitle(const QString& title);

  /**
   * @brief Adds a new value row.
   * @param name
   * @param value
   */
  void addValue(const QString& name, const QString& value);

  /**
   * @brief Adds a new spacer row
   */
  void addSpacer();

  /**
   * @brief Append the rows from another ToolTipGenerator to the end of this generator.
   * @param other
   */
  void append(const ToolTipGenerator& other);

  /**
   * @brief Clears the row data.
   */
  void clear();

  /**
   * @brief Returns the color name string.
   * @return
   */
  QString getRowColorStr() const;

  /**
   * @brief Sets the color name string.
   * @param colorStr
   */
  void setRowColorStr(const QString& colorStr);

  /**
   * @brief Generates and returns an HTML string representation.
   * @return
   */
  QString generateHTML() const;

protected:
  /**
   * @brief Creates and returns an HTML string for the given table row.
   * @param row
   * @return
   */
  QString rowToHTML(const RowItem& row) const;

private:
  QString m_RowColorStr = "#FFFCEA";
  std::vector<RowItem> m_Rows;
};
} // end namespace EbsdLib