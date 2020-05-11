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

#include <QtCore/QString>

/**
 * @file EbsdConstants.h
 * @brief This file contains many constants that are generic to the EBSD library
 */
namespace EbsdLib
{

using Rgb = uint32_t;
const Rgb RGB_MASK = 0x00ffffff; // masks RGB values
const QString PathSep("|");
static const uint8_t Unchecked = 0;
static const uint8_t PartiallyChecked = 1;
static const uint8_t Checked = 2;

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
const QString Statistics("Statistics");
const QString StatsData("StatsData");
const QString StatsType("StatsType");
const QString GBCD("GBCD");
} // namespace StringConstants

namespace NumericTypes
{
namespace Names
{
const QString Int8("signed   int 8  bit");
const QString UInt8("unsigned int 8  bit");
const QString Int16("signed   int 16 bit");
const QString UInt16("unsigned int 16 bit");
const QString Int32("signed   int 32 bit");
const QString UInt32("unsigned int 32 bit");
const QString Int64("signed   int 64 bit");
const QString UInt64("unsigned int 64 bit");
const QString Float("       Float 32 bit");
const QString Double("      Double 64 bit");
const QString Bool("Bool");
const QString SizeT("size_t");
} // namespace Names

enum class Type : int
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

const QString SupportedTypeList(NumericTypes::Names::Int8 + ", " + NumericTypes::Names::UInt8 + ", " + NumericTypes::Names::Int16 + ", " + NumericTypes::Names::UInt16 + ", " +
                                NumericTypes::Names::Int32 + ", " + NumericTypes::Names::UInt32 + ", " + NumericTypes::Names::Int64 + ", " + NumericTypes::Names::UInt64 + ", " +
                                NumericTypes::Names::Float + ", " + NumericTypes::Names::Double + ", " + NumericTypes::Names::Bool + ", " + NumericTypes::Names::SizeT);
} // namespace NumericTypes

/** @brief RefFrameZDir defined for the Stacking order of images into a 3D Volume */
namespace RefFrameZDir
{
static const unsigned int LowtoHigh = 0;
static const unsigned int HightoLow = 1;
static const unsigned int UnknownRefFrameZDirection = 2;
} // namespace RefFrameZDir

namespace H5Ebsd
{
const QString Manufacturer("Manufacturer");
const QString Header("Header");
const QString Phases("Phases");
const QString Phase("Phase");
const QString Data("Data");
const QString Index("Index");

const QString ZStartIndex("ZStartIndex");
const QString ZEndIndex("ZEndIndex");
const QString ZResolution("Z Resolution");
const QString StackingOrder("Stacking Order");
const QString SampleTransformationAngle("SampleTransformationAngle");
const QString SampleTransformationAxis("SampleTransformationAxis");
const QString EulerTransformationAngle("EulerTransformationAngle");
const QString EulerTransformationAxis("EulerTransformationAxis");

// Each Manufacturer has their own naming scheme for these variables but for
// DREAM.3D we are going to settle on using these names for consistency
const QString XResolution("X Resolution");
const QString YResolution("Y Resolution");

// We store the Maximum number of X and Y Points for the given volume. This
// allows us to store slices that have different XY voxel dimensions.
const QString XPoints("Max X Points");
const QString YPoints("Max Y Points");

const QString FileVersionStr("FileVersion");
const unsigned int FileVersion = 5;
const QString EbsdLibVersionStr("EbsdLibVersion");
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
const QString EulerAngles("EulerAngles");
const QString Phases("Phases");
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
const QString LowToHigh("Low To High");
const QString HighToLow("High To Low");
const QString UnknownStackingOrder("Unknown Stacking Order");

class Utils
{
public:
  static QString getStringForEnum(unsigned int v)
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

  static int getEnumForString(const QString& v)
  {
    if(EbsdLib::StackingOrder::LowToHigh.compare(v) == 0)
    {
      return EbsdLib::RefFrameZDir::LowtoHigh;
    }
    if(EbsdLib::StackingOrder::HighToLow.compare(v) == 0)
    {
      return EbsdLib::RefFrameZDir::HightoLow;
    }
    return EbsdLib::RefFrameZDir::UnknownRefFrameZDirection;
  }
};
} // namespace StackingOrder

/**
 * @brief IF YOU CHANGE THE VALUES THERE ARE DEEP RAMIFICATIONS IN THE CODE BUT
 * MOSTLY IN THE HDF5 FILES WHICH ARE WRITTEN USING THE ENUMERATIONS.
 */
namespace CrystalStructure
{

const unsigned int Triclinic = 4;       //!< Triclinic -1
const unsigned int Monoclinic = 5;      //!< Monoclinic 2/m
const unsigned int OrthoRhombic = 6;    //!< Orthorhombic mmm
const unsigned int Tetragonal_Low = 7;  //!< Tetragonal-Low 4/m
const unsigned int Tetragonal_High = 8; //!< Tetragonal-High 4/mmm
const unsigned int Trigonal_Low = 9;    //!< Trigonal-Low -3
const unsigned int Trigonal_High = 10;  //!< Trigonal-High -3m

const unsigned int Hexagonal_Low = 2;  //!< Hexagonal-Low 6/m
const unsigned int Hexagonal_High = 0; //!< Hexagonal-High 6/mmm
const unsigned int Cubic_Low = 3;      //!< Cubic Cubic-Low m3 (Tetrahedral)
const unsigned int Cubic_High = 1;     //!< Cubic Cubic-High m3m

const unsigned int LaueGroupEnd = 11;             //!< The end of the Laue groups
const unsigned int UnknownCrystalStructure = 999; //!< UnknownCrystalStructure
} // namespace CrystalStructure

namespace BravaisLattice
{
const QString Unknown("Unknown");
const QString Cubic("Cubic");
const QString Hexagonal("Hexagonal");
} // namespace BravaisLattice

namespace AngleRepresentation
{
static const int Radians = 0;
static const int Degrees = 1;
static const int Invalid = 2;
}; // namespace AngleRepresentation

namespace LambertParametersType
{
static const double iPi = 0.3183098861837910;   // 1/pi
static const double sPi = 1.7724538509055160;   // sqrt(pi)
static const double sPio2 = 1.2533141373155000; // sqrt(pi/2)
static const double sPi2 = 0.8862269254527580;  // sqrt(pi)/2
static const double srt = 0.8660254037844390;   // sqrt(3)/2
static const double isrt = 0.5773502691896260;  // 1/sqrt(3)
static const double alpha = 1.3467736870885980; // sqrt(pi)/3^(1/4)
static const double rtt = 1.7320508075688770;   // sqrt(3)
static const double prea = 0.5250375679043320;  // 3^(1/4)/sqrt(2pi)
static const double preb = 1.0500751358086640;  // 3^(1/4)sqrt(2/pi)
static const double prec = 0.9068996821171090;  // pi/2sqrt(3)
static const double pred = 2.0943951023931950;  // 2pi/3
static const double pree = 0.7598356856515930;  // 3^(-1/4)
static const double pref = 1.3819765978853420;  // sqrt(6/pi)
// ! the following constants are used for the cube to quaternion hemisphere mapping
static const double a = 1.9257490199582530;    // pi^(5/6)/6^(1/6)
static const double ap = 2.1450293971110250;   // pi^(2/3)
static const double sc = 0.8977727869612860;   // a/ap
static const double beta = 0.9628745099791260; // pi^(5/6)/6^(1/6)/2
static const double R1 = 1.3306700394914690;   //(3pi/4)^(1/3)
static const double r2 = 1.4142135623730950;   // sqrt(2)
static const double r22 = 0.7071067811865470;  // 1/sqrt(2)
static const double pi12 = 0.2617993877991490; // pi/12
static const double pi8 = 0.3926990816987240;  // pi/8
static const double prek = 1.6434564029725040; // R1 2^(1/4)/beta
static const double r24 = 4.8989794855663560;  // sqrt(24)
static const double tfit[16] = {1.00000000000188520,       -0.50000000021948470,     -0.0249999921275931260,    -0.0039287015447813740,     -0.00081527015354504380, -0.00020095004261197120,
                                -0.000023979867760717560,  -0.000082028689266058410, +0.000124487150420900920,  -0.00017491142148225770,    +0.00017034819341400540, -0.000120620650041168280,
                                +0.0000597197058686608260, -0.000019807567239656470, +0.0000039537146842128740, -0.000000365550014397195440};
static const double BP[6] = {0.0, 1.0, 0.5773502691896260, 0.4142135623730950, 0.0, 0.2679491924311230}; // used for Fundamental Zone determination in so3 module
} // namespace LambertParametersType

// Add some shortened namespace alias
// Condense some of the namespaces to same some typing later on.
namespace LPs = LambertParametersType;

} // namespace EbsdLib
