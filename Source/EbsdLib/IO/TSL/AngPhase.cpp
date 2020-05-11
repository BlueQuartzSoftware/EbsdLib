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

#include "AngPhase.h"
#include <cstring>

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AngPhase::AngPhase()
: m_PhaseIndex(-1)
, m_Symmetry(0)
, m_NumberFamilies(0)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AngPhase::~AngPhase() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::parseMaterialName(QList<QByteArray>& tokens)
{
  m_MaterialName.clear();
  for(int i = 1; i < tokens.size(); ++i)
  {
    m_MaterialName.append(tokens.at(i)).append(" ");
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::parseFormula(QList<QByteArray>& tokens)
{
  m_Formula.clear();
  for(int i = 1; i < tokens.size(); ++i)
  {
    m_Formula.append(tokens.at(i)).append(" ");
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::parseInfo(QList<QByteArray>& tokens)
{
  m_Info.clear();
  for(int i = 1; i < tokens.size(); ++i)
  {
    m_Info.append(tokens.at(i)).append(" ");
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
// void AngPhase::parseSymmetry(char* value, size_t start, size_t length)
//{
//  if (value[start] == ':')
//  {
//    ++start;
//  } // move past the ":" character
//  QByteArray data(&(value[start]), strlen(value) - start);
//  bool ok = false;
//  m_Symmetry = data.toUInt(&ok, 10);
//}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::parseLatticeConstants(QList<QByteArray>& tokens)
{
  m_LatticeConstants.clear();

  bool ok = false;
  m_LatticeConstants.push_back(tokens[1].toFloat(&ok)); // A
  m_LatticeConstants.push_back(tokens[2].toFloat(&ok)); // B
  m_LatticeConstants.push_back(tokens[3].toFloat(&ok)); // C
  m_LatticeConstants.push_back(tokens[4].toFloat(&ok)); // Alpha
  m_LatticeConstants.push_back(tokens[5].toFloat(&ok)); // Beta
  m_LatticeConstants.push_back(tokens[6].toFloat(&ok)); // Gamma
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
// void AngPhase::parseNumberFamilies(char* value, size_t start, size_t length)
//{
//  if (value[start] == ':')
//  {
//    ++start;
//  } // move past the ":" character
//  QByteArray data(&(value[start]), strlen(value) - start);
//  bool ok = false;
//  m_NumberFamilies = data.toInt(&ok, 10);
//}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::parseHKLFamilies(QList<QByteArray>& tokens)
{
  HKLFamily::Pointer family = HKLFamily::New();

  bool ok = false;
  family->h = tokens[1].toInt(&ok, 10);
  family->k = tokens[2].toInt(&ok, 10);
  family->l = tokens[3].toInt(&ok, 10);
  family->s1 = tokens[4].toInt(&ok, 10);
  family->diffractionIntensity = tokens[5].toFloat(&ok);
  if(tokens.size() > 6)
  {
    family->s2 = tokens[6].toInt(&ok, 10);
  }
  if(family->s1 > 1)
  {
    family->s1 = 1;
  }
  if(family->s2 > 1)
  {
    family->s2 = 1;
  }
  m_HKLFamilies.push_back(family);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::parseCategories(QList<QByteArray>& tokens)
{
  m_Categories.clear();
  bool ok = false;
  if(tokens[0].size() != EbsdLib::Ang::Categories.size())
  {
    tokens[0].replace(EbsdLib::Ang::Categories, "");
    m_Categories.push_back(tokens.at(0).toInt(&ok, 10));
  }
  for(int i = 1; i < tokens.size(); ++i)
  {
    m_Categories.push_back(tokens.at(i).toInt(&ok, 10));
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::printSelf(QTextStream& stream)
{
  stream << EbsdLib::Ang::Phase << ": " << m_PhaseIndex << QString("\n");
  stream << EbsdLib::Ang::MaterialName << ": " << m_MaterialName << QString("\n");
  stream << EbsdLib::Ang::Formula << ": " << m_Formula << QString("\n");
  stream << EbsdLib::Ang::Info << ": " << m_Info << QString("\n");
  stream << EbsdLib::Ang::Symmetry << ": " << m_Symmetry << QString("\n");

  stream << EbsdLib::Ang::LatticeConstants;

  for(const auto& latticeConstant : m_LatticeConstants)
  {
    stream << " " << latticeConstant;
  }
  stream << QString("\n");

  stream << EbsdLib::Ang::NumberFamilies << ": " << m_NumberFamilies << QString("\n");

  for(const auto& family : m_HKLFamilies)
  {
    family->printSelf(stream);
  }

  stream << EbsdLib::Ang::Categories;
  for(const auto& category : m_Categories)
  {
    stream << " " << category;
  }
  stream << QString("\n");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
unsigned int AngPhase::determineLaueGroup()
{
  uint32_t symmetry = getSymmetry();
  unsigned int crystal_structure = EbsdLib::CrystalStructure::UnknownCrystalStructure;

  switch(symmetry)
  {
  case EbsdLib::Ang::PhaseSymmetry::Cubic:
    crystal_structure = EbsdLib::CrystalStructure::Cubic_High;
    break;
  case EbsdLib::Ang::PhaseSymmetry::Tetrahedral:
    crystal_structure = EbsdLib::CrystalStructure::Cubic_Low;
    break;
  case EbsdLib::Ang::PhaseSymmetry::DiTetragonal:
    crystal_structure = EbsdLib::CrystalStructure::Tetragonal_High;
    break;
  case EbsdLib::Ang::PhaseSymmetry::Tetragonal:
    crystal_structure = EbsdLib::CrystalStructure::Tetragonal_Low;
    break;
  case EbsdLib::Ang::PhaseSymmetry::Orthorhombic:
    crystal_structure = EbsdLib::CrystalStructure::OrthoRhombic;
    break;
  case EbsdLib::Ang::PhaseSymmetry::Monoclinic_c:
  case EbsdLib::Ang::PhaseSymmetry::Monoclinic_b:
  case EbsdLib::Ang::PhaseSymmetry::Monoclinic_a:
    crystal_structure = EbsdLib::CrystalStructure::Monoclinic;
    break;
  case EbsdLib::Ang::PhaseSymmetry::Triclinic:
    crystal_structure = EbsdLib::CrystalStructure::Triclinic;
    break;
  case EbsdLib::Ang::PhaseSymmetry::DiHexagonal:
    crystal_structure = EbsdLib::CrystalStructure::Hexagonal_High;
    break;
  case EbsdLib::Ang::PhaseSymmetry::Hexagonal:
    crystal_structure = EbsdLib::CrystalStructure::Hexagonal_Low;
    break;
  case EbsdLib::Ang::PhaseSymmetry::DiTrigonal:
    crystal_structure = EbsdLib::CrystalStructure::Trigonal_High;
    break;
  case EbsdLib::Ang::PhaseSymmetry::Trigonal:
    crystal_structure = EbsdLib::CrystalStructure::Trigonal_Low;
    break;

  default:
    crystal_structure = EbsdLib::CrystalStructure::UnknownCrystalStructure;
  }
  return crystal_structure;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::setLatticeConstantA(float a)
{
  m_LatticeConstants[0] = a;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::setLatticeConstantB(float a)
{
  m_LatticeConstants[1] = a;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::setLatticeConstantC(float a)
{
  m_LatticeConstants[2] = a;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::setLatticeConstantAlpha(float a)
{
  m_LatticeConstants[3] = a;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::setLatticeConstantBeta(float a)
{
  m_LatticeConstants[4] = a;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngPhase::setLatticeConstantGamma(float a)
{
  m_LatticeConstants[5] = a;
}

// -----------------------------------------------------------------------------
HKLFamily::Pointer HKLFamily::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
AngPhase::Pointer AngPhase::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
void AngPhase::setMaterialName(const QString& value)
{
  m_MaterialName = value;
}

// -----------------------------------------------------------------------------
QString AngPhase::getMaterialName() const
{
  return m_MaterialName;
}

// -----------------------------------------------------------------------------
void AngPhase::setFormula(const QString& value)
{
  m_Formula = value;
}

// -----------------------------------------------------------------------------
QString AngPhase::getFormula() const
{
  return m_Formula;
}

// -----------------------------------------------------------------------------
void AngPhase::setInfo(const QString& value)
{
  m_Info = value;
}

// -----------------------------------------------------------------------------
QString AngPhase::getInfo() const
{
  return m_Info;
}

// -----------------------------------------------------------------------------
QString AngPhase::getNameOfClass() const
{
  return QString("AngPhase");
}

// -----------------------------------------------------------------------------
QString AngPhase::ClassName()
{
  return QString("AngPhase");
}

// -----------------------------------------------------------------------------
QString HKLFamily::getNameOfClass() const
{
  return QString("HKLFamily");
}

// -----------------------------------------------------------------------------
QString HKLFamily::ClassName()
{
  return QString("HKLFamily");
}
