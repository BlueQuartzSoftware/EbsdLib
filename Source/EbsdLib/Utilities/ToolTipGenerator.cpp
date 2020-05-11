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

#include "ToolTipGenerator.h"

#include <QtCore/QTextStream>

using namespace EbsdLib;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
ToolTipGenerator::ToolTipGenerator() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
ToolTipGenerator::~ToolTipGenerator()
{
  clear();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ToolTipGenerator::addTitle(const QString& title)
{
  RowItem titleRow;
  titleRow.type = RowItem::Type::Title;
  titleRow.name = title;
  m_Rows.push_back(titleRow);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ToolTipGenerator::addValue(const QString& name, const QString& value)
{
  RowItem valueRow;
  valueRow.type = RowItem::Type::Value;
  valueRow.name = name;
  valueRow.value = value;
  m_Rows.push_back(valueRow);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ToolTipGenerator::addSpacer()
{
  RowItem spacer;
  spacer.type = RowItem::Type::Spacer;
  m_Rows.push_back(spacer);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ToolTipGenerator::append(const ToolTipGenerator& other)
{
  addSpacer();
  for(const auto& row : other.m_Rows)
  {
    m_Rows.push_back(row);
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ToolTipGenerator::clear()
{
  m_Rows.clear();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString ToolTipGenerator::getRowColorStr() const
{
  return m_RowColorStr;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ToolTipGenerator::setRowColorStr(const QString& color)
{
  m_RowColorStr = color;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString ToolTipGenerator::generateHTML() const
{
  RowItem spacer;
  spacer.type = RowItem::Type::Spacer;

  QString html;
  QTextStream ss(&html);

  ss << "<html><head></head>\n";
  ss << "<body>\n";
  ss << "<table cellpadding=\"4\" cellspacing=\"0\" border=\"0\">\n";
  ss << "<tbody>\n";

  for(const auto& row : m_Rows)
  {
    ss << rowToHTML(row);
  }

  ss << rowToHTML(spacer);
  ss << "</tbody></table>\n";
  ss << "</body></html>";
  return html;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString ToolTipGenerator::rowToHTML(const RowItem& row) const
{
  QString html;
  QTextStream ss(&html);

  switch(row.type)
  {
  case RowItem::Type::Title:
    ss << R"(<tr bgcolor=")" << m_RowColorStr << R"("><th align="center" colspan=2>)" << row.name << "</th></tr>";
    break;
  case RowItem::Type::Value:
    ss << R"(<tr bgcolor=")" << m_RowColorStr << R"("><th align="right">)" << row.name << ":</th><td>" << row.value << "</td></tr>";
    break;
  case RowItem::Type::Spacer:
    ss << R"(<tr bgcolor=")" << m_RowColorStr << R"("><td></td><td></td></tr>)";
    break;
  }

  return html;
}
