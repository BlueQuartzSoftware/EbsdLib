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

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "EbsdLib/Core/EbsdSetGetMacros.h"
#include "EbsdLib/EbsdLib.h"

//#pragma pack(push, r1, 1) /* push current alignment to stack. set alignment to 1 byte boundary */
/*!
 * @struct HKLFamily_t is used to write the HKL Family to an HDF5 file using a
 * compound data type.
 */
struct HKLFamily_t
{
  int h;
  int k;
  int l;
  float diffractionIntensity;
  char s1;
  char s2;
};

/**
 * @class HKLFamily HKLFamily.h EbsdLib/IO/TSL/HKLFamily.h
 * @brief Class to hold the information associated with a HKL Family value
 *
 * @date Mar 23, 2011
 * @version 1.0
 */
class EbsdLib_EXPORT HKLFamily
{
public:
  using Self = HKLFamily;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;

  static Pointer NullPointer();

  HKLFamily();
  //  HKLFamily(const HKLFamily&) = delete;            // Copy Constructor Not Implemented
  //  HKLFamily(HKLFamily&) = delete;                  // Move Constructor Not Implemented
  //  HKLFamily& operator=(const HKLFamily&) = delete; // Copy Assignment Not Implemented
  //  HKLFamily& operator=(HKLFamily&) = delete;       // Move Assignment Not Implemented
  virtual ~HKLFamily();

  EBSD_STATIC_NEW_MACRO(HKLFamily)

  /**
   * @brief Returns the name of the class for HKLFamily
   */

  std::string getNameOfClass() const;

  /**
   * @brief Returns the name of the class for HKLFamily
   */
  static std::string ClassName();

  int h = 0;
  int k = 0;
  int l = 0;
  float diffractionIntensity = 0.0F;
  char s1 = '\0';
  char s2 = '\0';

  /**
   * @brief Prints this class to the output stream. Useful for debuggin
   * @param stream The stream to print to
   */
  void printSelf(std::stringstream& stream) const;

  /**
   * @brief Copies the content of this instance to another class instance
   * @param ptr The destination of the copied contents
   */
  void copyToStruct(HKLFamily_t* ptr);

  /**
   * @brief Copies the content <b>from</b> another structure into this structure
   * @param ptr The source of the copy, ie, the values from <i>ptr</i> will be copied
   * into this instance.
   */
  void copyFromStruct(HKLFamily_t* ptr);
};

/**
 * @class AngPhase AngPhase.h EbsdLib/IO/TSL/AngPhase.h
 * @brief This class holds all the values for a "Phase" header block in a TSL file
 *
 * @date Mar 23, 2011
 * @version 1.0
 */
class EbsdLib_EXPORT AngPhase
{
public:
  using Self = AngPhase;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<Self>;
  static Pointer NullPointer();

  AngPhase();
  //  AngPhase(const AngPhase&) = delete;            // Copy Constructor Not Implemented
  //  AngPhase(AngPhase&) = delete;                  // Move Constructor Not Implemented
  //  AngPhase& operator=(const AngPhase&) = delete; // Copy Assignment Not Implemented
  //  AngPhase& operator=(AngPhase&) = delete;       // Move Assignment Not Implemented
  virtual ~AngPhase();

  EBSD_STATIC_NEW_MACRO(AngPhase)

  /**
   * @brief Returns the name of the class for AngPhase
   */
  std::string getNameOfClass() const;

  /**
   * @brief Returns the name of the class for AngPhase
   */
  static std::string ClassName();

  EBSD_INSTANCE_PROPERTY(int, PhaseIndex)

  /**
   * @brief Setter property for MaterialName
   */
  void setMaterialName(const std::string& value);
  /**
   * @brief Getter property for MaterialName
   * @return Value of MaterialName
   */
  std::string getMaterialName() const;

  /**
   * @brief Setter property for Formula
   */
  void setFormula(const std::string& value);
  /**
   * @brief Getter property for Formula
   * @return Value of Formula
   */
  std::string getFormula() const;

  /**
   * @brief Setter property for Info
   */
  // void setInfo(const std::string& value);
  /**
   * @brief Getter property for Info
   * @return Value of Info
   */
  // std::string getInfo() const;

  EBSD_INSTANCE_PROPERTY(uint32_t, Symmetry)
  EBSD_INSTANCE_PROPERTY(std::vector<float>, LatticeConstants)
  EBSD_INSTANCE_PROPERTY(int, NumberFamilies)
  EBSD_INSTANCE_PROPERTY(std::vector<HKLFamily::Pointer>, HKLFamilies)
  EBSD_INSTANCE_PROPERTY(std::vector<int>, Categories)

  void setLatticeConstantA(float a);
  void setLatticeConstantB(float a);
  void setLatticeConstantC(float a);
  void setLatticeConstantAlpha(float a);
  void setLatticeConstantBeta(float a);
  void setLatticeConstantGamma(float a);

  void parseMaterialName(std::vector<std::string>& tokens);
  void parseFormula(std::vector<std::string>& tokens);
  // void parseInfo(std::vector<std::string>& tokens);
  void parseLatticeConstants(std::vector<std::string>& tokens);
  void parseHKLFamilies(std::vector<std::string>& tokens);
  void parseCategories(std::vector<std::string>& tokens);

  void printSelf(std::stringstream& stream);

  /**
   * @brief Returns the type of crystal structure for this phase.
   */
  unsigned int determineLaueGroup();

private:
  std::string m_MaterialName = {};
  std::string m_Formula = {};
  // std::string m_Info = {};
};

//#pragma pack(pop, r1)
