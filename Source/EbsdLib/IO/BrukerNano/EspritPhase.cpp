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

#include "EspritPhase.h"

#include <cstring>

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EspritPhase::EspritPhase()
: m_PhaseIndex(-1)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EspritPhase::~EspritPhase() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string EspritPhase::getMaterialName()
{
  return m_Name;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
unsigned int EspritPhase::determineLaueGroup()
{
  int sg = getIT();

  if(sg >= 1 && sg <= 2)
  {
    return EbsdLib::CrystalStructure::Triclinic;
  }

  if(sg >= 3 && sg <= 15)
  {
    return EbsdLib::CrystalStructure::Monoclinic;
  }

  if(sg >= 16 && sg <= 74)
  {
    return EbsdLib::CrystalStructure::OrthoRhombic;
  }

  if(sg >= 75 && sg <= 89)
  {
    return EbsdLib::CrystalStructure::Tetragonal_Low;
  }
  if(sg >= 90 && sg <= 142)
  {
    return EbsdLib::CrystalStructure::Tetragonal_High;
  }

  if(sg >= 143 && sg <= 148)
  {
    return EbsdLib::CrystalStructure::Trigonal_Low;
  }
  if(sg >= 149 && sg <= 167)
  {
    return EbsdLib::CrystalStructure::Trigonal_High;
  }

  if(sg >= 168 && sg <= 176)
  {
    return EbsdLib::CrystalStructure::Hexagonal_Low;
  }
  if(sg >= 177 && sg <= 194)
  {
    return EbsdLib::CrystalStructure::Hexagonal_High;
  }

  if(sg >= 195 && sg <= 206)
  {
    return EbsdLib::CrystalStructure::Cubic_Low;
  }
  if(sg >= 207 && sg <= 230)
  {
    return EbsdLib::CrystalStructure::Cubic_High;
  }

  return EbsdLib::CrystalStructure::UnknownCrystalStructure;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void EspritPhase::parseFormula(std::vector<std::string>& tokens)
{
  m_Formula.clear();
  for(const auto& token : tokens)
  {
    m_Formula += token + " ";
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void EspritPhase::parseName(std::vector<std::string>& tokens)
{
  m_Name.clear();
  for(const auto& token : tokens)
  {
    m_Name += token + " ";
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void EspritPhase::parseSpaceGroup(std::vector<std::string>& tokens)
{
  m_SpaceGroup.clear();
  for(const auto& token : tokens)
  {
    m_SpaceGroup += token + " ";
  }
}

// -----------------------------------------------------------------------------
EspritPhase::Pointer EspritPhase::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
void EspritPhase::setFormula(const std::string& value)
{
  m_Formula = value;
}

// -----------------------------------------------------------------------------
std::string EspritPhase::getFormula() const
{
  return m_Formula;
}

// -----------------------------------------------------------------------------
void EspritPhase::setName(const std::string& value)
{
  m_Name = value;
}

// -----------------------------------------------------------------------------
std::string EspritPhase::getName() const
{
  return m_Name;
}

// -----------------------------------------------------------------------------
void EspritPhase::setSpaceGroup(const std::string& value)
{
  m_SpaceGroup = value;
}

// -----------------------------------------------------------------------------
std::string EspritPhase::getSpaceGroup() const
{
  return m_SpaceGroup;
}

// -----------------------------------------------------------------------------
std::string EspritPhase::getNameOfClass() const
{
  return std::string("EspritPhase");
}

// -----------------------------------------------------------------------------
std::string EspritPhase::ClassName()
{
  return std::string("EspritPhase");
}
