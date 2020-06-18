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

#include "CtfPhase.h"

#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
CtfPhase::CtfPhase()
: m_PhaseIndex(-1)
, m_PhaseName("-1")
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
CtfPhase::~CtfPhase() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void CtfPhase::convertEuropeanDecimals(std::string& line)
{
  // Filter the line to convert European command style decimals to US/UK style points
  for(char& c : line)
  {
    if(c == ',')
    {
      c = '.';
    }
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void CtfPhase::parsePhase(const std::string& line)
{
  EbsdStringUtils::StringTokenType tokens = EbsdStringUtils::split(line, '\t');

  convertEuropeanDecimals(tokens[0]);
  convertEuropeanDecimals(tokens[1]);
  // The lattice constants and lattice angles are ';' delimited
  m_LatticeConstants.resize(6);
  // The first set of values are the lattice constants

  EbsdStringUtils::StringTokenType latConst = EbsdStringUtils::split(tokens[0], ';');
  m_LatticeConstants[0] = std::stof(latConst[0]);
  m_LatticeConstants[1] = std::stof(latConst[1]);
  m_LatticeConstants[2] = std::stof(latConst[2]);
  // The second set of values are the lattice angles
  latConst = EbsdStringUtils::split(tokens[1], ';');
  m_LatticeConstants[3] = std::stof(latConst[0]);
  m_LatticeConstants[4] = std::stof(latConst[1]);
  m_LatticeConstants[5] = std::stof(latConst[2]);

  // 3rd set is the Name of the phase
  m_PhaseName = tokens[2];

  // 4th set is the Symmetry group
  m_LaueGroup = static_cast<EbsdLib::Ctf::LaueGroupTable>(std::stoi(tokens[3]));

  if(tokens.size() == 5)
  {
    m_Comment = tokens[4];
  }
  else if(tokens.size() == 8)
  {
    m_SpaceGroup = std::stoi(tokens[4]);
    m_Internal1 = tokens[5];
    m_Internal2 = tokens[6];
    m_Comment = tokens[7];
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void CtfPhase::printSelf(std::ostream& stream)
{
  stream << EbsdLib::Ctf::LatticeConstants << " " << m_LatticeConstants[0] << ", " << m_LatticeConstants[1] << ", " << m_LatticeConstants[2] << " " << m_LatticeConstants[3] << ", "
         << m_LatticeConstants[4] << ", " << m_LatticeConstants[5] << std::endl;
  stream << EbsdLib::Ctf::PhaseName << " " << m_PhaseName << std::endl;
  stream << EbsdLib::Ctf::LaueGroup << " " << m_LaueGroup << std::endl;
  stream << EbsdLib::Ctf::SpaceGroup << " " << m_SpaceGroup << std::endl;
  stream << EbsdLib::Ctf::Internal1 << " " << m_Internal1 << std::endl;
  stream << EbsdLib::Ctf::Internal2 << " " << m_Internal2 << std::endl;
  stream << EbsdLib::Ctf::Comment << " " << m_Comment << std::endl;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
unsigned int CtfPhase::determineLaueGroup()
{
  EbsdLib::Ctf::LaueGroupTable symmetry = getLaueGroup();

  switch(symmetry)
  {
  case EbsdLib::Ctf::LG_Triclinic:
    return EbsdLib::CrystalStructure::Triclinic;
  case EbsdLib::Ctf::LG_Monoclinic:
    return EbsdLib::CrystalStructure::Monoclinic;
  case EbsdLib::Ctf::LG_Orthorhombic:
    return EbsdLib::CrystalStructure::OrthoRhombic;
  case EbsdLib::Ctf::LG_Tetragonal_Low:
    return EbsdLib::CrystalStructure::Tetragonal_Low;
  case EbsdLib::Ctf::LG_Tetragonal_High:
    return EbsdLib::CrystalStructure::Tetragonal_High;
  case EbsdLib::Ctf::LG_Trigonal_Low:
    return EbsdLib::CrystalStructure::Trigonal_Low;
  case EbsdLib::Ctf::LG_Trigonal_High:
    return EbsdLib::CrystalStructure::Trigonal_High;
  case EbsdLib::Ctf::LG_Hexagonal_Low:
    return EbsdLib::CrystalStructure::Hexagonal_Low;
  case EbsdLib::Ctf::LG_Hexagonal_High:
    return EbsdLib::CrystalStructure::Hexagonal_High;
  case EbsdLib::Ctf::LG_Cubic_Low:
    return EbsdLib::CrystalStructure::Cubic_Low;
  case EbsdLib::Ctf::LG_Cubic_High:
    return EbsdLib::CrystalStructure::Cubic_High;
  case EbsdLib::Ctf::UnknownSymmetry:
    return EbsdLib::CrystalStructure::UnknownCrystalStructure;
  default:
    break;
  }
  return EbsdLib::CrystalStructure::UnknownCrystalStructure;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string CtfPhase::getMaterialName()
{
  return m_PhaseName;
}

// -----------------------------------------------------------------------------
CtfPhase::Pointer CtfPhase::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
void CtfPhase::setPhaseName(const std::string& value)
{
  m_PhaseName = value;
}

// -----------------------------------------------------------------------------
std::string CtfPhase::getPhaseName() const
{
  return m_PhaseName;
}

// -----------------------------------------------------------------------------
void CtfPhase::setInternal1(const std::string& value)
{
  m_Internal1 = value;
}

// -----------------------------------------------------------------------------
std::string CtfPhase::getInternal1() const
{
  return m_Internal1;
}

// -----------------------------------------------------------------------------
void CtfPhase::setInternal2(const std::string& value)
{
  m_Internal2 = value;
}

// -----------------------------------------------------------------------------
std::string CtfPhase::getInternal2() const
{
  return m_Internal2;
}

// -----------------------------------------------------------------------------
void CtfPhase::setComment(const std::string& value)
{
  m_Comment = value;
}

// -----------------------------------------------------------------------------
std::string CtfPhase::getComment() const
{
  return m_Comment;
}

// -----------------------------------------------------------------------------
std::string CtfPhase::getNameOfClass() const
{
  return std::string("CtfPhase");
}

// -----------------------------------------------------------------------------
std::string CtfPhase::ClassName()
{
  return std::string("CtfPhase");
}
