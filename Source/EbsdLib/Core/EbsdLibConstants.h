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

#include <string>

/**
 * @file EbsdConstants.h
 * @brief This file contains many constants that are generic to the EBSD library
 */
namespace EbsdLib
{

using Rgb = uint32_t;
inline constexpr Rgb RGB_MASK = 0x00ffffff; // masks RGB values
inline const std::string PathSep("|");
inline constexpr uint8_t Unchecked = 0;
inline constexpr uint8_t PartiallyChecked = 1;
inline constexpr uint8_t Checked = 2;

enum InfoStringFormat
{
  HtmlFormat = 0,
  MarkDown = 1,
  //      JsonFormat,
  //      TextFormat,
  //      XmlFormat,
  UnknownFormat
};

namespace StringConstants
{
inline const std::string Statistics("Statistics");
inline const std::string StatsData("StatsData");
inline const std::string StatsType("StatsType");
inline const std::string GBCD("GBCD");
} // namespace StringConstants

namespace NumericTypes
{
namespace Names
{
inline const std::string Int8("signed int 8 bit");
inline const std::string UInt8("unsigned int 8 bit");
inline const std::string Int16("signed int 16 bit");
inline const std::string UInt16("unsigned int 16 bit");
inline const std::string Int32("signed int 32 bit");
inline const std::string UInt32("unsigned int 32 bit");
inline const std::string Int64("signed int 64 bit");
inline const std::string UInt64("unsigned int 64 bit");
inline const std::string Float("Float 32 bit");
inline const std::string Double("Double 64 bit");
inline const std::string Bool("Bool");
inline const std::string SizeT("size_t");
} // namespace Names

enum class Type : int32_t
{
  Int8 = 0,
  UInt8,
  Int16,
  UInt16,
  Int32,
  UInt32,
  Int64,
  UInt64,
  Float,
  Double,
  Bool,
  SizeT,
  UnknownNumType
};

inline const std::string SupportedTypeList(NumericTypes::Names::Int8 + ", " + NumericTypes::Names::UInt8 + ", " + NumericTypes::Names::Int16 + ", " + NumericTypes::Names::UInt16 + ", " +
                                           NumericTypes::Names::Int32 + ", " + NumericTypes::Names::UInt32 + ", " + NumericTypes::Names::Int64 + ", " + NumericTypes::Names::UInt64 + ", " +
                                           NumericTypes::Names::Float + ", " + NumericTypes::Names::Double + ", " + NumericTypes::Names::Bool + ", " + NumericTypes::Names::SizeT);
} // namespace NumericTypes

/** @brief RefFrameZDir defined for the Stacking order of images into a 3D Volume */
namespace RefFrameZDir
{
inline constexpr uint32_t LowtoHigh = 0;
inline constexpr uint32_t HightoLow = 1;
inline constexpr uint32_t UnknownRefFrameZDirection = 2;
} // namespace RefFrameZDir

namespace H5Ebsd
{
inline const std::string Manufacturer("Manufacturer");
inline const std::string Header("Header");
inline const std::string Phases("Phases");
inline const std::string Phase("Phase");
inline const std::string Data("Data");
inline const std::string Index("Index");

inline const std::string ZStartIndex("ZStartIndex");
inline const std::string ZEndIndex("ZEndIndex");
inline const std::string ZResolution("Z Resolution");
inline const std::string StackingOrder("Stacking Order");
inline const std::string SampleTransformationAngle("SampleTransformationAngle");
inline const std::string SampleTransformationAxis("SampleTransformationAxis");
inline const std::string EulerTransformationAngle("EulerTransformationAngle");
inline const std::string EulerTransformationAxis("EulerTransformationAxis");

// Each Manufacturer has their own naming scheme for these variables but for
// DREAM.3D we are going to settle on using these names for consistency
inline const std::string XResolution("X Resolution");
inline const std::string YResolution("Y Resolution");

// We store the Maximum number of X and Y Points for the given volume. This
// allows us to store slices that have different XY voxel dimensions.
inline const std::string XPoints("Max X Points");
inline const std::string YPoints("Max Y Points");

inline const std::string FileVersionStr("FileVersion");
inline constexpr uint32_t FileVersion = 5;
inline const std::string EbsdLibVersionStr("EbsdLibVersion");
} // namespace H5Ebsd

using EnumType = int32_t;
enum class OEM : EnumType
{
  EDAX = 0,
  Oxford = 1,
  Bruker = 2,
  HEDM = 3,
  Zeiss = 4,
  Phillips = 5,
  ThermoFisher = 6,
  DREAM3D = 7,
  Unknown = 8
};

namespace CellData
{
inline const std::string EulerAngles("EulerAngles");
inline const std::string Phases("Phases");
} // namespace CellData

enum EbsdToSampleCoordinateMapping
{
  TSLdefault = 0,
  HKLdefault = 1,
  HEDMdefault = 2,
  UnknownCoordinateMapping = 3
};

namespace StackingOrder
{
inline const std::string LowToHigh("Low To High");
inline const std::string HighToLow("High To Low");
inline const std::string UnknownStackingOrder("Unknown Stacking Order");

namespace Utils
{
inline std::string getStringForEnum(uint32_t v)
{
  if(EbsdLib::RefFrameZDir::LowtoHigh == v)
  {
    return EbsdLib::StackingOrder::LowToHigh;
  }
  if(EbsdLib::RefFrameZDir::HightoLow == v)
  {
    return EbsdLib::StackingOrder::HighToLow;
  }
  return EbsdLib::StackingOrder::UnknownStackingOrder;
}

inline int32_t getEnumForString(const std::string& v)
{
  if(EbsdLib::StackingOrder::LowToHigh == v)
  {
    return EbsdLib::RefFrameZDir::LowtoHigh;
  }
  if(EbsdLib::StackingOrder::HighToLow == v)
  {
    return EbsdLib::RefFrameZDir::HightoLow;
  }
  return EbsdLib::RefFrameZDir::UnknownRefFrameZDirection;
}
} // namespace Utils
} // namespace StackingOrder

/**
 * @brief IF YOU CHANGE THE VALUES THERE ARE DEEP RAMIFICATIONS IN THE CODE BUT
 * MOSTLY IN THE HDF5 FILES WHICH ARE WRITTEN USING THE ENUMERATIONS.
 */
namespace CrystalStructure
{

inline constexpr uint32_t Triclinic = 4;       //!< Triclinic -1
inline constexpr uint32_t Monoclinic = 5;      //!< Monoclinic 2/m
inline constexpr uint32_t OrthoRhombic = 6;    //!< Orthorhombic mmm
inline constexpr uint32_t Tetragonal_Low = 7;  //!< Tetragonal-Low 4/m
inline constexpr uint32_t Tetragonal_High = 8; //!< Tetragonal-High 4/mmm
inline constexpr uint32_t Trigonal_Low = 9;    //!< Trigonal-Low -3
inline constexpr uint32_t Trigonal_High = 10;  //!< Trigonal-High -3m

inline constexpr uint32_t Hexagonal_Low = 2;  //!< Hexagonal-Low 6/m
inline constexpr uint32_t Hexagonal_High = 0; //!< Hexagonal-High 6/mmm
inline constexpr uint32_t Cubic_Low = 3;      //!< Cubic Cubic-Low m3 (Tetrahedral)
inline constexpr uint32_t Cubic_High = 1;     //!< Cubic Cubic-High m3m

inline constexpr uint32_t LaueGroupEnd = 11;             //!< The end of the Laue groups
inline constexpr uint32_t UnknownCrystalStructure = 999; //!< UnknownCrystalStructure
} // namespace CrystalStructure

namespace BravaisLattice
{
inline const std::string Unknown("Unknown");
inline const std::string Cubic("Cubic");
inline const std::string Hexagonal("Hexagonal");
} // namespace BravaisLattice

namespace AngleRepresentation
{
inline constexpr int32_t Radians = 0;
inline constexpr int32_t Degrees = 1;
inline constexpr int32_t Invalid = 2;
}; // namespace AngleRepresentation

namespace LambertParametersType
{
inline constexpr double iPi = 0.3183098861837910;   // 1/pi
inline constexpr double sPi = 1.7724538509055160;   // sqrt(pi)
inline constexpr double sPio2 = 1.2533141373155000; // sqrt(pi/2)
inline constexpr double sPi2 = 0.8862269254527580;  // sqrt(pi)/2
inline constexpr double srt = 0.8660254037844390;   // sqrt(3)/2
inline constexpr double isrt = 0.5773502691896260;  // 1/sqrt(3)
inline constexpr double alpha = 1.3467736870885980; // sqrt(pi)/3^(1/4)
inline constexpr double rtt = 1.7320508075688770;   // sqrt(3)
inline constexpr double prea = 0.5250375679043320;  // 3^(1/4)/sqrt(2pi)
inline constexpr double preb = 1.0500751358086640;  // 3^(1/4)sqrt(2/pi)
inline constexpr double prec = 0.9068996821171090;  // pi/2sqrt(3)
inline constexpr double pred = 2.0943951023931950;  // 2pi/3
inline constexpr double pree = 0.7598356856515930;  // 3^(-1/4)
inline constexpr double pref = 1.3819765978853420;  // sqrt(6/pi)
// ! the following constants are used for the cube to quaternion hemisphere mapping
inline constexpr double a = 1.9257490199582530;    // pi^(5/6)/6^(1/6)
inline constexpr double ap = 2.1450293971110250;   // pi^(2/3)
inline constexpr double sc = 0.8977727869612860;   // a/ap
inline constexpr double beta = 0.9628745099791260; // pi^(5/6)/6^(1/6)/2
inline constexpr double R1 = 1.3306700394914690;   //(3pi/4)^(1/3)
inline constexpr double r2 = 1.4142135623730950;   // sqrt(2)
inline constexpr double r22 = 0.7071067811865470;  // 1/sqrt(2)
inline constexpr double pi12 = 0.2617993877991490; // pi/12
inline constexpr double pi8 = 0.3926990816987240;  // pi/8
inline constexpr double prek = 1.6434564029725040; // R1 2^(1/4)/beta
inline constexpr double r24 = 4.8989794855663560;  // sqrt(24)
inline constexpr double tfit[16] = {1.00000000000188520,       -0.50000000021948470,     -0.0249999921275931260,    -0.0039287015447813740,     -0.00081527015354504380, -0.00020095004261197120,
                                    -0.000023979867760717560,  -0.000082028689266058410, +0.000124487150420900920,  -0.00017491142148225770,    +0.00017034819341400540, -0.000120620650041168280,
                                    +0.0000597197058686608260, -0.000019807567239656470, +0.0000039537146842128740, -0.000000365550014397195440};
inline constexpr double BP[6] = {0.0, 1.0, 0.5773502691896260, 0.4142135623730950, 0.0, 0.2679491924311230}; // used for Fundamental Zone determination in so3 module
} // namespace LambertParametersType

// Add some shortened namespace alias
// Condense some of the namespaces to same some typing later on.
namespace LPs = LambertParametersType;

} // namespace EbsdLib
