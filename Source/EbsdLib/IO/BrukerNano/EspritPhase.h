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

#include <array>
#include <string>
#include <sstream>
#include <vector>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdSetGetMacros.h"

class EbsdLib_EXPORT EspritPhase
{
public:
  using Self = EspritPhase;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer();

  EBSD_STATIC_NEW_MACRO(EspritPhase)
  /**
   * @brief Returns the name of the class for EspritPhase
   */
  std::string getNameOfClass() const;
  /**
   * @brief Returns the name of the class for EspritPhase
   */
  static std::string ClassName();

  virtual ~EspritPhase();

  EBSD_INSTANCE_PROPERTY(int, PhaseIndex)

  /**
   * @brief Setter property for Formula
   */
  void setFormula(const std::string& value);
  /**
   * @brief Getter property for Formula
   * @return Value of Formula
   */
  std::string getFormula() const;

  EBSD_INSTANCE_PROPERTY(int, IT)
  EBSD_INSTANCE_PROPERTY(std::vector<float>, LatticeConstants)
  /**
   * @brief Setter property for Name
   */
  void setName(const std::string& value);
  /**
   * @brief Getter property for Name
   * @return Value of Name
   */
  std::string getName() const;

  EBSD_INSTANCE_PROPERTY(int, Setting)
  /**
   * @brief Setter property for SpaceGroup
   */
  void setSpaceGroup(const std::string& value);
  /**
   * @brief Getter property for SpaceGroup
   * @return Value of SpaceGroup
   */
  std::string getSpaceGroup() const;

  std::string getMaterialName();

  void parseFormula(std::vector<std::string>& tokens);
  void parseName(std::vector<std::string>& tokens);
  void parseSpaceGroup(std::vector<std::string>& tokens);

  void printSelf(std::stringstream& stream);

  /**
   * @brief Returns the type of crystal structure for this phase.
   */
  unsigned int determineLaueGroup();

protected:
  EspritPhase();

private:
  std::string m_Name = {};
  std::string m_MaterialName = {};
  std::string m_SpaceGroup = {};
  std::string m_Formula = {};

public:
  EspritPhase(const EspritPhase&) = delete;            // Copy Constructor Not Implemented
  EspritPhase(EspritPhase&&) = delete;                 // Move Constructor Not Implemented
  EspritPhase& operator=(const EspritPhase&) = delete; // Copy Assignment Not Implemented
  EspritPhase& operator=(EspritPhase&&) = delete;      // Move Assignment Not Implemented
};

